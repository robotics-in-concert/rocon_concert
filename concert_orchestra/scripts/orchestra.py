#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/hydro-devel/concert_orchestra/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import concert_orchestra

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('orchestration')  # , log_level=rospy.DEBUG)
    orchestration = concert_orchestra.Orchestration()
    rospy.spin()
