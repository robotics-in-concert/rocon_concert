#!/usr/bin/env python

import rospy
import traceback
import threading
import concert_msgs.srv as concert_srv
import concert_msgs.msg as concert_msg
import rocon_std_msgs.msg as rocon_std_msg
import rocon_app_manager_msgs.srv as rapp_mamanager_srvs
import scheduler_msgs.msg as scheduler_msg
import uuid_msgs.msg as uuid_msg
import unique_id


class ConcertServiceInstance(object):

    description = None
    pub = {}
    sub = {}

    def __init__(self, service_description):
        '''
            service_desciption : ConcertService.msg
        '''
        # Setting description(msg)
        self.description      = service_description
        self.description.uuid = unique_id.toMsg(unique_id.fromRandom())
        self.loginfo("UUID = " + str(self.description.uuid))

        self.setup_ros_api()

        # Enable/Disable
#        self.enable(self.description.enabled)

    def __del__(self):
        self.shutdown()

    def setup_ros_api(self):
        self.pub['request_resources'] = rospy.Publisher('request_resources',concert_msg.RequestResources)


    def shutdown(self):
        self.loginfo("Destroying...")
        self.loginfo("Disabling... service")
        self.disable()
        self.loginfo("Shutting down subscriber")
# self.sub['list_concert_clients'].unregister()
        self.loginfo("Destroying...done")

    def enable(self):
        self.loginfo("Enabled")

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
        self.loginfo("Disabled")

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
        
    def loginfo(self,msg):
        rospy.loginfo("Service["+str(self.description.name)+"] : " + str(msg))
