#! /usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################
import rospy
from concert_software_manager import Manager

if __name__ == '__main__':
    rospy.init_node('software_manager')
    
    manager = Manager()
    manager.loginfo("Initialized")
    manager.spin()
    manager.loginfo("Bye Bye")
