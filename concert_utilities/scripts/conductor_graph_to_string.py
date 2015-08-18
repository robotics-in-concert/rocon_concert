#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
###############################################################################
import rospy
from concert_utilities.conductor_graph import ConductorGraph2Dot

if __name__ == '__main__':

    rospy.init_node('conductor_graph_to_string')
    
    cgds = ConductorGraph2Dot()
    cgds.spin()
