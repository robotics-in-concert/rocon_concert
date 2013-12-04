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
import concert_msgs.msg as concert_msgs

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('scheduler')
    scheduler = concert_scheduler.DemoScheduler(concert_msgs.Strings.SCHEDULER_REQUESTS)
    scheduler.spin()
