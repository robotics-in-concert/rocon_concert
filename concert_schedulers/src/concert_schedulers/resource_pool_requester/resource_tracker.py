#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import unique_id
from rocon_utilities import platform_tuples

##############################################################################
# Methods
##############################################################################


class ResourceTracker(object):
    __slots__ = [
            'resource',            # the object we want to track
            'high_priority_flag',  # if tracking, boolean for whether it is high or low priority
            'tracking',            # it is part of a currently issued request
            'allocated',           # it is tracking and has been allocated by the scheduler
            'name',                # name of the resource, for easy reference (ros_package/rapp name)
            'platform_tuple'       # platform info of this resource, for easy reference (rocon_std_msgs/PlatformTuple)
        ]

    def __init__(self, resource):
        self.resource = resource

        # aliases
        self.name = self.resource.name
        self.platform_tuple = self.resource.platform_tuple
        self.reset_scheduler_flags()

    def __str__(self):
        """ Generate string representation. """
        return 'Resource Tracker' \
            + '\n    key: ' + str(self.key()) \
            + '\n    name: ' + str(self.name) \
            + '\n    platform_tuple: ' + platform_tuples.to_string(self.platform_tuple) \
            + '\n    tracking|allocated|high_priority: ' + str(self.tracking) + "|" + str(self.allocated) + "|" + str(self.high_priority_flag)

    def key(self):
        return unique_id.toHexString(self.resource.id)

    def reset_scheduler_flags(self):
        self.allocated = False
        self.tracking = False
        self.high_priority_flag = False
