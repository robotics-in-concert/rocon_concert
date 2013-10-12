#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
import rospy
import concert_roles

if __name__ == '__main__':

    rospy.init_node('role_manager')
    role_manager = concert_roles.RoleManager()
    role_manager.spin()
