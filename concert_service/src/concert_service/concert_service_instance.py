#!/usr/bin/env python

import rospy
import traceback
import threading
import concert_msgs.srv as concert_srv
import concert_msgs.msg as concert_msg
import rocon_std_msgs.msg as rocon_std_msg
import rocon_app_manager_msgs.srv as rapp_mamanager_srvs

from .compatibility_tree import *
from .exceptions import *


class ConcertServiceInstance(object):

    description = None
    pub = {}

    def __init__(self, service_description):
        '''
            service_desciption : ConcertService.msg
        '''
        # Setting description(msg)
        self.description = service_description
        self.setup_ros_api()

        # Enable/Disable
#        self.enable(self.description.enabled)

    def __del__(self):
        self.shutdown()

    def setup_ros_api(self):
        self.pub['request_resources'] = rospy.Publisher('request_resources',concert_msg.RequestResources)


    def shutdown(self):
        self.log("Destroying...")
        self.log("Disabling... service")
        self.disable()
        self.log("Shutting down subscriber")
# self.sub['list_concert_clients'].unregister()
        self.log("Destroying...done")

    def enable(self):
        self.log("Enabled")

        if self.description.enabled:
            return False, "already enabled"

        self.description.enabled = True

        # Request to allocate resource 
        msg = concert_msg.RequestResources()
        msg.service_name = self.description.name
        msg.linkgraph = self.description.linkgraph
        msg.enable = True

        self.pub['request_resources'].publish(msg)
        
        return True, ""

    def disable(self):
        self.log("Disabled")

        if not self.description.enabled:
            return False, "already disabled"

        self.description.enabled = False

        # Request to cancel resource 
        msg = concert_msg.RequestResources()
        msg.service_name = self.description.name
        msg.linkgraph = self.description.linkgraph
        msg.enable = False

        self.pub['request_resources'].publish(msg)

        return True, ""

    def to_msg(self):
        return self.description
        
    def log(self,msg):
        rospy.loginfo("Service["+str(self.description.name)+"] : " + str(msg))
