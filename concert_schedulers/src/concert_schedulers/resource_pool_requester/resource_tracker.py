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
            'resource',
            'high_priority_flag',
            'tracking',
            'allocated',
            'name',
            'platform_info'
        ]

    def __init__(self, resource):
        self.resource = resource

        # aliases
        self.name = self.resource.name
        self.platform_info = self.resource.platform_info
        self.reset_scheduler_flags()

    def __str__(self):
        """ Generate string representation. """
        return 'Resource Tracker' \
            + '\n    key: ' + str(self.key()) \
            + '\n    name: ' + str(self.name) \
            + '\n    platform_info: ' + str(self.platform_info) \
            + '\n    tracking|allocated|high_priority: ' + str(self.tracking) + "|" + str(self.allocated) + "|" + str(self.high_priority_flag)

    def key(self):
        return unique_id.toHexString(self.resource.id)

    def reset_scheduler_flags(self):
        self.allocated = False
        self.tracking = False
        self.high_priority_flag = False
