#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
import rospy
import concert_master

if __name__ == '__main__':

    rospy.init_node('master')
    concert_master = concert_master.ConcertMaster()
    concert_master.spin()
