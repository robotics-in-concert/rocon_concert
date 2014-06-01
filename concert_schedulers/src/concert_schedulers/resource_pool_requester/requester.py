#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
"""
.. module:: resource_pool_requester.requester

The resource pool requester.
"""
##############################################################################
# Imports
##############################################################################

import threading

import rospy
import unique_id
import concert_scheduler_requests
import scheduler_msgs.msg as scheduler_msgs
import concert_msgs.msg as concert_msgs
import rocon_uri

##############################################################################
# Methods
##############################################################################


def request_completely_unallocated(request):
    '''
      Quick check to see if a request's resources have been entirely
      lost (i.e. unallocated). This will trigger the responder to
      cleanup (i.e. release the request).

      :param request:
      :type request: scheduler_msgs.Request

      :returns: true or false if entirely unallocated or not.
      :rtype: bool
    '''
    for resource in request.msg.resources:
        if rocon_uri.parse(resource.uri).name.string != concert_msgs.Strings.SCHEDULER_UNALLOCATED_RESOURCE:
            return False
    return True

##############################################################################
# Classes
##############################################################################


class ResourcePoolRequester(object):
    '''
      A variant on the basic requester that can handle some higher level
      formulation and management of requests.
    '''
    __slots__ = [
            '_requester',        # the base requester class from concert_scheduler_requests
            '_resource_groups',
            '_state',            # state of the requester (see State class)
            '_feedback',
            '_high_priority',    # priority to set for necessary (minimum) resource requirements
            '_low_priority',     # priority to set for optional resource requirements
            '_recovery_start',   # the timestamp recorded when moving from State.ALIVE -> State.RECOVERING (used for recovery timeout)
            '_lock'
        ]

    class State(object):
        PENDING = 'pending'             # pending resource allocations for minimum requirements of the resource groups
        ALIVE = 'alive'                 # minimum resource requirements have been allocated - it is now functioning
        RECOVERING = 'recovering'       # was alive, but resource allocations fell below minimum requirements and trying to recover
        timeout = rospy.Duration(10.0)  # timeout for dropping 'alive' state status.

    def __init__(self,
                 resource_groups,
                 feedback,
                 high_priority=scheduler_msgs.Request.HIGH_PRIORITY,
                 low_priority=scheduler_msgs.Request.LOW_PRIORITY,
                 uuid=None,
                 topic=concert_scheduler_requests.common.SCHEDULER_TOPIC,
                 frequency=concert_scheduler_requests.common.HEARTBEAT_HZ):
        '''
          Loads the requester up with a collecation of resource groups that have min/max requirements
          for each resource type.

          :param resource_groups: many resource groups that will form the bases for this requester
          :type resource_groups: resource_group.MinMaxResourceGroup[]

          :param func feedback: external feedback callback function
          :param int high_priority: priority to set for necessary (minimum) resource requirements
          :param int low_priority: priority to set for optional resource requirements
        '''
        self._requester = concert_scheduler_requests.Requester(self._requester_feedback, uuid, 0, topic, frequency)
        self._resource_groups = resource_groups
        self._feedback = feedback
        self._state = self.State.PENDING
        self._high_priority = high_priority
        self._low_priority = low_priority
        self._lock = threading.Lock()
        self._issue_minimum_request()
        self._requester.send_requests()

    def _issue_minimum_request(self):
        initial_resources = []
        for resource_group in self._resource_groups:
            initial_resources.extend(resource_group.initial_resources())
        unused_minimum_request_uuid = self._requester.new_request(initial_resources, priority=self._high_priority)

    def cancel_all_requests(self):
        '''
          Exactly as it says! Used typically when shutting down or when
          it's lost more allocated resources than the minimum required (in which case it
          cancels everything and starts reissuing new requests).
        '''
        #self._lock.acquire()
        rospy.loginfo("Requester : cancelling all requests")
        self._requester.cancel_all()
        self._requester.send_requests()
        #self._lock.release()

    def _requester_feedback(self, request_set):
        '''
          This returns requests processed by the scheduler with whatever necessary modifications
          that were made on the original requests.

          @param request_set : the modified requests
          @type dic { uuid.UUID : scheduler_msgs.ResourceRequest }
        '''
        # call self._feedback in here
        #print("Request set: %s" % request_set)
        ########################################
        # Update all resource tracking info
        ########################################
        for resource_group in self._resource_groups:
            resource_group.reset_scheduler_flags()
        for request in request_set.values():
            high_priority_flag = True if request.msg.priority == self._high_priority else False
            if request.msg.status == scheduler_msgs.Request.NEW or request.msg.status == scheduler_msgs.Request.WAITING:
                self._flag_resource_trackers(request.msg.resources, tracking=True, allocated=False, high_priority_flag=high_priority_flag)
            elif request.msg.status == scheduler_msgs.Request.GRANTED:
                self._flag_resource_trackers(request.msg.resources, tracking=True, allocated=True, high_priority_flag=high_priority_flag)
                if request_completely_unallocated(request):
                    rospy.loginfo("Requester : cancelling request [has been completely unallocated]")
                    request.cancel()
            elif request.msg.status == scheduler_msgs.Request.CLOSED:
                self._flag_resource_trackers(request.msg.resources, tracking=False, allocated=False)

        #for resource_group in self._resource_groups:
        #    print("\n%s" % str(resource_group))

        ########################################
        # Requester state transitions
        ########################################
        tentatively_alive = True
        for resource_group in self._resource_groups:
            if not resource_group.is_alive():
                tentatively_alive = False
                break
        # alive state changes
        if self._state == self.State.PENDING and tentatively_alive:
            self._state = self.State.ALIVE
            rospy.loginfo("Requester : state change [%s->%s]" % (self.State.PENDING, self.State.ALIVE))
        elif self._state == self.State.ALIVE and not tentatively_alive:
            rospy.loginfo("Requester : state change [%s->%s]" % (self.State.ALIVE, self.State.RECOVERING))
            self._state = self.State.RECOVERING
            self._recovery_start = rospy.Time.now()
            # Not practical, but currently the safe non-thread way.
            #self._requester.cancel_all()
            #self._issue_minimum_request()
            # maybe not such a great idea - the requester is not officially thread-safe,
            # but it's interesting to test the problems with doing this
            thread = threading.Thread(target=self._check_recovery_timeout)
            thread.start()
        elif self._state == self.State.RECOVERING and tentatively_alive:
            self._state = self.State.ALIVE
        # else moving from RECOVERING to PENDING is handled by the thread function

        ########################################
        # Check optional request requirements
        ########################################
        if self._state == self.State.ALIVE:
            for resource_group in self._resource_groups:
                (resource, high_priority_flag) = resource_group.requires_new_request()
                if resource is not None:
                    priority = self._high_priority if high_priority_flag else self._low_priority
                    unused_request_uuid = self._requester.new_request([resource], priority=priority)

    def _check_recovery_timeout(self):
        '''
          Thread worker function that monitors the requester state while it's trying to
          recover the minimum necessary requirements to run the resource pool. If
          nothing happens for the duration of this wait, then it cancels all requests (deallocating
          any resources) and reissues brand new requests as needed.

          In the future this probably has to change as we can't dictate to the service how long
          it should wait before cancelling requests. Ultimately the service itself should handle
          what happens if this timeout is reached - it needs to cleanup (i.e. finalise
          whatever it is doing first such as sending robots back to home base) before finally issuing
          the cancel order for the request.
        '''
        while (rospy.Time.now() - self._recovery_start) < self.State.timeout and not rospy.is_shutdown() and self._state == self.State.RECOVERING:
            rospy.rostime.wallsleep(1.0)
            if (rospy.Time.now() - self._recovery_start) > self.State.timeout:
                self.cancel_all_requests()
                rospy.logwarn("Requester : timed out trying to recover necessary resource pool state.")
                rospy.logwarn("Requester : issuing new minimum request.")
                self._issue_minimum_request()
                self._state = self.State.PENDING

    def _flag_resource_trackers(self, resources, tracking, allocated, high_priority_flag=False):
        '''
          Update the flags in the resource trackers for each resource. This is used to
          follow whether a particular resource is getting tracked, or is already allocated so
          that we can determine if new requests should be made and at what priority.
        '''
        for resource in resources:
            # do a quick check to make sure individual resources haven't been previously allocated, and then lost
            # WARNING : this unallocated check doesn't actually work - the requester isn't sending us back this info yet.
            if rocon_uri.parse(resource.uri).name.string == concert_msgs.Strings.SCHEDULER_UNALLOCATED_RESOURCE:
                tracking = False
                allocated = False
            resource_tracker = self._find_resource_tracker(unique_id.toHexString(resource.id))
            if resource_tracker is None:
                pass  # should raise an exception
            else:
                resource_tracker.tracking = tracking
                resource_tracker.allocated = allocated
                resource_tracker.high_priority_flag = high_priority_flag

    def _find_resource_tracker(self, key):
        '''
          @param key : unique identifier for the resource
          @type hex string

          @return the resource tracker corresponding to the key
          @type ResourceTracker or None
        '''
        for resource_group in self._resource_groups:
            resource_tracker = resource_group.find_resource_tracker(key)
            if resource_tracker is not None:
                return resource_tracker
        return None
