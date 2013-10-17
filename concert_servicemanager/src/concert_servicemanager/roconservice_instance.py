#!/usr/bin/env python

import rospy

class RoconServiceInstance(object):


    def __init__(self):
        self.log("Hola")

    def log(self,msg):
        rospy.log("[Put name] : ")
