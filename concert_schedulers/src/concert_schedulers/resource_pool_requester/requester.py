#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import copy
import rospy
import unique_id
import rocon_scheduler_requests
import scheduler_msgs.msg as scheduler_msgs
import concert_msgs.msg as concert_msgs
import threading
import rocon_utilities

##############################################################################
# Methods
##############################################################################


class ResourcePoolRequester(object):
    '''
      A variant on the basic requester that can handle some higher level
      formulation and management of requests.
    '''
    __slots__ = [
            '_requester',        # the base requester class from rocon_scheduler_requests
            '_resource_groups',
            '_state',            # state of the requester (see State class)
            '_feedback',
            '_high_priority',    # priority to set for necessary (minimum) resource requirements
            '_low_priority',     # priority to set for optional resource requirements
            '_initial_request_uuid',
            '_request_set',
            '_lock'
        ]

    class State(object):
        PENDING = 'pending'        # pending resource allocations for minimum requirements of the resource groups
        ALIVE = 'alive'            # minimum resource requirements have been allocated - it is now functioning
        RECOVERING = 'recovering'  # was alive, but resource allocations fell below minimum requirements and trying to recover

    def __init__(self,
                 resource_groups,
                 feedback,
                 high_priority=10,
                 low_priority=0,
                 uuid=None,
                 topic=rocon_scheduler_requests.common.SCHEDULER_TOPIC,
                 frequency=rocon_scheduler_requests.common.HEARTBEAT_HZ):
        '''
          Loads the requester up with a collecation of resource groups that have min/max requirements
          for each resource type.

          @param resource_groups : many resource groups that will form the bases for this requester
          @type resource_group.MinMaxResourceGroup[]

          @param feedback : external feedback callback function
          @type method

          @param high_priority : priority to set for necessary (minimum) resource requirements
          @type int

          @param low_priority : priority to set for optional resource requirements
          @type int
        '''
        self._requester = rocon_scheduler_requests.Requester(self._requester_feedback, uuid, 0, topic, frequency)
        self._resource_groups = resource_groups
        self._feedback = feedback
        self._state = self.State.PENDING
        self._high_priority = high_priority
        self._low_priority = low_priority
        self._request_set = None
        self._lock = threading.Lock()

        initial_resources = []
        for resource_group in self._resource_groups:
            initial_resources.extend(resource_group.initial_resources())
        self._initial_request_uuid = self._requester.new_request(initial_resources, priority=self._high_priority)

    def cancel_all_requests(self):
        '''
          Exactly as it says! Used typically when shutting down.

          Note - called by external processes, so make sure it's protected.
        '''
        #self._lock.acquire()
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
        self._lock.acquire()
        self._request_set = copy.deepcopy(request_set)
        self._lock.release()
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

        #for resource_group in self._resource_groups:
        #    print("\n%s" % str(resource_group))

        ########################################
        # Update requester pool state
        ########################################
        tentatively_alive = True
        for resource_group in self._resource_groups:
            if not resource_group.is_alive():
                tentatively_alive = False
                break
        # alive state changes
        if self._state == self.State.PENDING and tentatively_alive:
            rospy.loginfo("Requester : gone live.")
            self._state = self.State.ALIVE
        elif self._state == self.State.ALIVE and not tentatively_alive:
            self._state = self.State.PENDING
            # @todo : should cancel all requests and issue a brand new one
            # or have a timeout state that gives it a chance to recover alive status
            # before cancelling.
            rospy.logwarn("Requester : we died.")
        ########################################
        # Check optional request requirements
        ########################################
        if self._state == self.State.ALIVE:
            for resource_group in self._resource_groups:
                (resource, high_priority_flag) = resource_group.requires_new_request()
                if resource is not None:
                    priority = self._high_priority if high_priority_flag else self._low_priority
                    unused_request_uuid = self._requester.new_request([resource], priority=priority)

    def _flag_resource_trackers(self, resources, tracking, allocated, high_priority_flag):
        '''
          Update the flags in the resource trackers for each resource. This is used to
          follow whether a particular resource is getting tracked, or is already allocated so
          that we can determine if new requests should be made and at what priority.
        '''
        for resource in resources:
            # do a quick check to make sure individual resources haven't been previously allocated, and then lost
            # WARNING : this unallocated check doesn't actually work - the requester isn't sending us back this info yet.
            if rocon_utilities.platform_info.get_name(resource.platform_info) == concert_msgs.Strings.SCHEDULER_UNALLOCATED_RESOURCE:
                rospy.logwarn("Requester : unallocated resource %s" % resource.platform_info)
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
