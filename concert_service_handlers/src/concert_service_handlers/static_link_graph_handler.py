#!/usr/bin/env python

import rospy

import std_msgs
import concert_msgs.msg as concert_msg

class StaticLinkGraphHandler(object):
    linkgraph = None

    pub = {}
    sub = {}

    def __init__(self, name, linkgraph):
        self.name = name
        self.linkgraph = linkgraph

        self.setup_ros_api()

    def setup_ros_api(self):
        self.pub['request_resources'] = rospy.Publisher('/concert/request_resources', concert_msg.RequestResources)
        self.pub['test'] = rospy.Publisher('test',std_msgs.msg.String)

    def request_resources(self, enable):

        rospy.loginfo("Enable : " + str(enable))

        msg = concert_msg.RequestResources()
        msg.service_name = self.name
        msg.linkgraph = self.linkgraph
        msg.enable = enable 

        self.pub['request_resources'].publish(msg)
        rospy.loginfo("Here")
        
    def spin(self):
        rospy.sleep(3)
        self.request_resources(True)
        rospy.loginfo("Requested")
        rospy.spin()

        rospy.loginfo("Hola!")
        self.request_resources(False)
        rospy.sleep(3)
