#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
import rospy
import concert_conductor

if __name__ == '__main__':

    rospy.init_node('conductor')
    conductor = concert_conductor.Conductor()
    conductor.spin()
