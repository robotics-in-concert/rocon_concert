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
# Classes
##############################################################################


class StaticLinkGraphHandler(object):

    def __init__(self, name, linkgraph):
        self.init_variables(name, linkgraph)
        self.setup_ros_api()

    def init_variables(self, name, linkgraph):
        self.name = name
        self.linkgraph = linkgraph

        self.pub = {}
        self.sub = {}

    def setup_ros_api(self):
        self.pub['request_resources'] = rospy.Publisher(concert_msgs.Strings.REQUEST_RESOURCES, concert_msgs.RequestResources, latch=True)

    def request_resources(self, enable):

        rospy.loginfo("enable : " + str(enable))

        msg = concert_msgs.RequestResources()
        msg.service_name = self.name
        msg.linkgraph = self.linkgraph
        msg.enable = enable

        self.pub['request_resources'].publish(msg)

    def shutdown(self):
        self.request_resources(False)

    def spin(self):
        self.request_resources(True)
        rospy.on_shutdown(self.shutdown)  # shutdown hook
        rospy.spin()
