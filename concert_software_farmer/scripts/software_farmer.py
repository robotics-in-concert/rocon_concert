#! /usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################
import rospy
from concert_software_farmer import SoftwareFarmer

if __name__ == '__main__':
    rospy.init_node('software_farmer')
    
    farmer = SoftwareFarmer()
    farmer.spin()
