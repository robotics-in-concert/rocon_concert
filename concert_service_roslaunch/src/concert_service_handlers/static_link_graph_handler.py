#!/usr/bin/env python

import rospy

import std_msgs
import concert_msgs.msg as concert_msg

class StaticLinkGraphHandler(object):


    def __init__(self, name, linkgraph):
        self.init_variables(name,linkgraph)
        self.setup_ros_api()

    def init_variables(self, name, linkgraph):
        self.name = name
        self.linkgraph = linkgraph

        self.pub = {}
        self.sub = {}

    def setup_ros_api(self):
        self.pub['request_resources'] = rospy.Publisher(conert_msg.Strings.REQUEST_RESOURCES, concert_msg.RequestResources)

    def request_resources(self, enable):

        rospy.loginfo("Enable : " + str(enable))

        msg = concert_msg.RequestResources()
        msg.service_name = self.name
        msg.linkgraph = self.linkgraph
        msg.enable = enable 

        self.pub['request_resources'].publish(msg)
        rospy.loginfo("Here")

    def shutdown(self):
        self.request_resources(False)
        
    def spin(self):
        rospy.sleep(3)
        self.request_resources(True)
        rospy.loginfo("Requested")
        rospy.on_shutdown(self.shutdown) # shutdown hook
        rospy.spin()

        rospy.loginfo("Hola!")
        rospy.sleep(3)
