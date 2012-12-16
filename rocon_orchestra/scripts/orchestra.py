#!/usr/bin/env python

import roslib
roslib.load_manifest('rocon_orchestra')
import rospy
import rocon_orchestra

if __name__ == '__main__':
    rospy.init_node('orchestration')  # , log_level=rospy.DEBUG)
    orchestration = rocon_orchestra.Orchestration()
    rospy.spin()
