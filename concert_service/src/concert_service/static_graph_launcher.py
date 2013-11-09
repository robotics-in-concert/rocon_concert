#!/usr/bin/env python

import rospy

class StaticLinkGraphLauncher(object)

    linkgraph = None

    pub = {}
    sub = {}

    def __init__(self,linkgraph):
        self.linkgraph = linkgraph
        
    def spin(self):
        rospy.spin()
         
