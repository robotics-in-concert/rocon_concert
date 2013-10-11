#! /usr/bin/env python

import rospy
from op_msgs.srv import *

rospy.init_node('add_srv')

filename = rospy.get_param('~filename')
rospy.wait_for_service('add_a_service_from_file')
srv = rospy.ServiceProxy('add_a_service_from_file',AddServiceFromFile)

r = srv(filename)
rospy.loginfo(str(r))
