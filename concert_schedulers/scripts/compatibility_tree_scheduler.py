#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import concert_schedulers
import concert_msgs.msg as concert_msgs

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('scheduler')
    # If we use the periodic publisher, then we can use those interrupts as the loop to do continuous checking but it might be
    # an idea to optimise this better so it isn't continually processing
    scheduler = concert_schedulers.CompatibilityTreeScheduler(concert_msgs.Strings.CONCERT_CLIENTS, "~requests")
    scheduler.spin()
