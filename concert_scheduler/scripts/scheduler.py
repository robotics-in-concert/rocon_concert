#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import concert_scheduler

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('scheduler')
    scheduler = concert_scheduler.ConcertScheduler()
    scheduler.spin()
