#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import unique_id

##############################################################################
# Methods
##############################################################################


class ResourceTracker(object):
    __slots__ = [
            'resource',            # the object we want to track
            'high_priority_flag',  # if tracking, boolean for whether it is high or low priority
            'tracking',            # it is part of a currently issued request
            'allocated',           # it is tracking and has been allocated by the scheduler
            'rapp',                # name of the resource (rapp), for easy reference (ros_package/rapp name)
            'uri'                  # rocon uri representation of this resource for easy reference (rocon uri string)
        ]

    def __init__(self, resource):
        '''
          @param resource : resource object to track.
          @type scheduler_msgs.Resource
        '''
        self.resource = resource

        # aliases
        self.rapp = self.resource.rapp
        self.uri = self.resource.uri
        self.reset_scheduler_flags()

    def __str__(self):
        """ Generate string representation. """
        return 'Resource Tracker' \
            + '\n    key: ' + str(self.key()) \
            + '\n    name: ' + str(self.rapp) \
            + '\n    uri: ' + self.uri \
            + '\n    tracking|allocated|high_priority: ' + str(self.tracking) + "|" + str(self.allocated) + "|" + str(self.high_priority_flag)

    def key(self):
        return unique_id.toHexString(self.resource.id)

    def reset_scheduler_flags(self):
        self.allocated = False
        self.tracking = False
        self.high_priority_flag = False
