#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

#import rospy
import rocon_scheduler_requests

##############################################################################
# Methods
##############################################################################


class Requester():
    '''
      A variant on the basic requester that can handle some higher level
      formulation and management of requests.
    '''
    __slots__ = [
            '_requester',  # the base requester class from rocon_scheduler_requests
        ]

    def __init__(self, feedback, uuid=None,
                 priority=0,
                 topic=rocon_scheduler_requests.common.SCHEDULER_TOPIC,
                 frequency=rocon_scheduler_requests.common.HEARTBEAT_HZ):
        self._requester = rocon_scheduler_requests.Requester(feedback, uuid, priority, topic, frequency)
