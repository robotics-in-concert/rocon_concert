#!/usr/bin/env python

import rospy

class ServiceManager(object):
    def __init__(self):
        self.log("in init")
        
    def log(self,msg):
        rospy.loginfo("Serivce Manger : " + str(msg))

    def spin(self):
        self.log("in spin")
        rospy.spin()
