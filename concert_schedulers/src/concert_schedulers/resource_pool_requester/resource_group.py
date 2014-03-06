#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import copy
import unique_id

# local imports
from concert_schedulers.common.exceptions import InvalidResourceGroupException
from .resource_tracker import ResourceTracker

##############################################################################
# Classes
##############################################################################


class ResourcePoolGroup(object):

    __slots__ = [
            '_resources',  # resources this group must care for.
            '_min',
            '_max',
        ]

    def __init__(self, minimum, resources):
        '''
          Constructor fully initialises this request.

          @param minimum
          @type int

          @param resources : dict of resources eligible for use in constructing scheduler requests
          @type { uuid hexstring : scheduler_msgs.Resource }
        '''
        self._resources = {}
        for resource in resources:
            resource.id = unique_id.toMsg(unique_id.fromRandom())
            key = unique_id.toHexString(resource.id)
            self._resources[key] = ResourceTracker(resource)
        self._min = minimum
        self._max = len(resources)
        self._validate()  # can raise an exception

    def get_resource_trackers(self):
        return self._resources.values()

    def __str__(self):
        """ Generate string representation. """
        string_representation = '=== Resource Group ===\n'
        for resource_tracker in self._resources.values():
            string_representation += '\n' + str(resource_tracker)
        return string_representation

    def is_alive(self):
        count = 0
        for resource_tracker in self._resources.values():
            if resource_tracker.allocated:
                count += 1
        return False if count < self._min else True

    def requires_new_request(self):
        '''
          Checks to see if we should make offer a new one-resource request (basically
          when # tracking = # allocated >= minimum. Note, it also checks priority flags
          to determine whether it should request a high or low priority.
        '''
        tracking_and_allocated = 0
        high_priority_trackers = 0
        free_resource_trackers = []
        unallocated_tracker_pending = False
        for resource_tracker in self._resources.values():
            if resource_tracker.tracking and not resource_tracker.allocated:
                unallocated_tracker_pending = True
            if resource_tracker.tracking:
                tracking_and_allocated += 1
                if resource_tracker.high_priority_flag:
                    high_priority_trackers += 1
            else:
                free_resource_trackers.append(copy.deepcopy(resource_tracker))
        # rospy.loginfo("Requester : length of free_resource_trackers: [%s][%s][%s]" % (tracking_and_allocated, high_priority_trackers, len(free_resource_trackers)))
        if unallocated_tracker_pending or not free_resource_trackers:
            return (None, False)
        resource = free_resource_trackers[0].resource
        if high_priority_trackers < self._min:
            return (resource, True)
        else:
            return (resource, False)

    def _validate(self):
        '''
          Check that the stored resources are all of the same type:
          i.e. name, uri (not parameters or remappings)
        '''
        if self._min < 0:
            raise InvalidResourceGroupException("Requester : attempted to create invalid min-max request [%s < 0]" % self._min)
        if len(self._resources) < self._min:
            raise InvalidResourceGroupException("Requester : attempted to create invalid min-max request [ %s < %s(min)]" % (len(self._resources.keys()), self._min))
        template = self._resources.values()[0]
        for resource in self._resources.values():
            if template.rapp != resource.rapp:
                raise InvalidResourceGroupException("Requester : attempted to create invalid min-max request [%s != %s]" % (template.rapp, resource.rapp))
            if template.uri != resource.uri:
                raise InvalidResourceGroupException("Requester : attempted to create invalid min-max request [%s != %s]" % (template.uri, resource.uri))

    def initial_resources(self):
        '''
          Set of resources that would make up an initial request - this needs to be the necessary
          level in order to satisfy the higher level requirements of this type of request (in this
          case the minimum boundary).

          @return list of resources for the request.
          @rtype scheduler_msgs.Resource
        '''
        resources = [resource_tracker.resource for resource_tracker in self._resources.values()]
        return resources[:self._min]

    def reset_scheduler_flags(self):
        for resource_tracker in self._resources.values():
            resource_tracker.reset_scheduler_flags()

    def find_resource_tracker(self, key):
        if key in self._resources.keys():
            return self._resources[key]
        else:
            return None
#        for resource_tracker in self._resources.values():
#            if resource_tracker.key() == key:
#                return resource_tracker
#        return None
