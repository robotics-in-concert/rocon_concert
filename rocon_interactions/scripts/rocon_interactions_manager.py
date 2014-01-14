#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
import rospy
import rocon_interactions

if __name__ == '__main__':

    rospy.init_node('rocon_interactions')
    role_manager = rocon_interactions.RoleManager()
    role_manager.spin()
