#!/bin/env python

'''
The orchestrator.

@author: Daniel Stonier
'''
##############################################################################
# Imports
##############################################################################

import roslib
roslib.load_manifest('rocon_orchestra')
import rospy
import concert_msgs.msg as concert_msgs

# Local imports
from .implementation import Implementation

##############################################################################
# Callbacks
##############################################################################


class Orchestration(object):

    def __init__(self):
        self._implementation = Implementation()
        rospy.Subscriber("list_concert_clients", concert_msgs.ConcertClients, self._callback_concert_clients)

    def _callback_concert_clients(self, concert):
        '''
          The conductor publishes the concert client list, which also happens to
          be latched so you'll always get the latest list.
        '''
        for concert_client in concert.clients:
            rospy.loginfo("Orchestration: updated concert clients list:")
            rospy.loginfo("       Client: %s" % (concert_client.name))
            rospy.loginfo("               %s.%s.%s" % (concert_client.platform, concert_client.system, concert_client.robot))
            rospy.loginfo("               %s" % concert_client.client_status)
