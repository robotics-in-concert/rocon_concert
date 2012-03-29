#!/bin/env python

'''
The orchestrator.

@author: Daniel Stonier
'''
##############################################################################
# Imports
##############################################################################

import sys
#import xmlrpclib

# Ros imports
import roslib; roslib.load_manifest('rocon_orchestra')
import rospy

from concert_comms.msg import ConcertClients

# Local imports
from .implementation import Implementation

##############################################################################
# Callbacks
##############################################################################

def callback_concert_clients(concert):
    '''
      The conductor publishes the concert client list, which also happens to
      be latched so you'll always get the latest list.
    '''
    for concert_client in concert.clients:
        rospy.loginfo("Orchestration: concert clients list:")
        rospy.loginfo("       Client: %s [%s]"%(concert_client.unique_name,concert_client.suggested_name))
        rospy.loginfo("               %s.%s.%s"%(concert_client.platform,concert_client.system,concert_client.robot))
        if concert_client.is_connected:
            rospy.loginfo("               connected")
        else:
            rospy.loginfo("               not connected")

##############################################################################
# Main
##############################################################################

def main():
    rospy.init_node('orchestration', log_level=rospy.DEBUG)
    rospy.sleep(1.0)
    rospy.Subscriber("concert_clients", ConcertClients, callback_concert_clients)
    unused_loaded_device_configuration = Implementation()
    rospy.spin()
    return 0

