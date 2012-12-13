#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_orchestration/concert_master/LICENSE
#
##############################################################################
# Imports
##############################################################################

import roslib
roslib.load_manifest('concert_master')
import rospy
from concert_master.concert_master import ConcertMaster

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('concert_master')

    cm = ConcertMaster()
    rospy.loginfo("Initialised")
    cm.spin()
