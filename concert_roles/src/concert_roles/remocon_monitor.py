#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import concert_msgs.msg as concert_msgs

##############################################################################
# Remocon Monitor
##############################################################################


class RemoconMonitor(object):
    '''
      Attaches a subscriber to a remocon publisher and monitors the
      status of the remocon.
    '''
    __slots__ = [
            'name',
            'status',  # concert_msgs.RemoconStatus
            '_subscriber',
        ]

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self, topic_name):
        if topic_name.startswith('/remocons/'):
            self.name = topic_name[10:]
        else:
            self.name = 'unknown'  # should raise an error here
            return
        self._subscriber = rospy.Subscriber(topic_name, concert_msgs.RemoconStatus, self._callback)
        self.status = None

    def _callback(self, msg):
        self.status = msg

    def unregister(self):
        self._subscriber.unregister()
