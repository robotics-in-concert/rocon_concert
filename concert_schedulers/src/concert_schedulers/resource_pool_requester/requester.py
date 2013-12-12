#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import unique_id
import rocon_scheduler_requests
import scheduler_msgs.msg as scheduler_msgs

##############################################################################
# Methods
##############################################################################


class ResourceTracker(object):
    __slots__ = [
            '_resource',
        ]

    def __init__(self, resource):
        self._resource = resource


class ResourcePoolRequester():
    '''
      A variant on the basic requester that can handle some higher level
      formulation and management of requests.
    '''
    __slots__ = [
            '_requester',  # the base requester class from rocon_scheduler_requests
            '_resource_groups',
            '_feedback',
            '_high_priority',
            '_low_priority',
            '_initial_request_uuid',
            'alive'
        ]

    def __init__(self,
                 resource_groups,
                 feedback,
                 high_priority=10,
                 low_priority=0,
                 uuid=None,
                 topic=rocon_scheduler_requests.common.SCHEDULER_TOPIC,
                 frequency=rocon_scheduler_requests.common.HEARTBEAT_HZ):
        '''
          @param resource_groups : many resource groups that will form a single request
          @type resource_group.MinMaxResourceGroup[]
        '''
        self._requester = rocon_scheduler_requests.Requester(self._requester_feedback, uuid, 0, topic, frequency)
        self._resource_groups = resource_groups
        self._feedback = feedback
        self._alive = False
        self._high_priority = high_priority
        self._low_priority = low_priority

        initial_resources = []
        for resource_group in self._resource_groups:
            initial_resources.extend(resource_group.initial_resources())
        self._initial_request_uuid = self._requester.new_request(initial_resources, priority=self._high_priority)

    def _requester_feedback(self, request_set):
        # call self._feedback in here
        #print("Request set: %s" % request_set)
        # should we reset all tracking, allocated flags in all resources here, then enable them below?
        for resource_group in self._resource_groups:
            resource_group.reset_scheduler_flags()
        for request in request_set.values():
            high_priority_flag = True if request.msg.priority == self._high_priority else False
            if request.msg.status == scheduler_msgs.Request.NEW:
                self._flag_resource_trackers(request.msg.resources, tracking=True, allocated=False, high_priority_flag=high_priority_flag)
            elif request.msg.status == scheduler_msgs.Request.GRANTED:
                self._flag_resource_trackers(request.msg.resources, tracking=True, allocated=True, high_priority_flag=high_priority_flag)

        #for resource_group in self._resource_groups:
        #    print("\n%s" % str(resource_group))

        # check alive status
        tentatively_alive = True
        for resource_group in self._resource_groups:
            if not resource_group.is_alive():
                tentatively_alive = False
                break
        # alive state changes
        if not self._alive and tentatively_alive:
            rospy.loginfo("Requester : gone live.")
            self._alive = True
        elif self._alive and not tentatively_alive:
            self._alive = False
            # should cancel all requests and issue a new one.
            rospy.logwarn("Requester : we died.")
        if self._alive:
            for resource_group in self._resource_groups:
                (resource, high_priority_flag) = resource_group.requires_new_request()
                if resource is not None:
                    priority = self._high_priority if high_priority_flag else self._low_priority
                    unused_request_uuid = self._requester.new_request([resource], priority=priority)

    def _flag_resource_trackers(self, resources, tracking, allocated, high_priority_flag):
        for resource in resources:
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
