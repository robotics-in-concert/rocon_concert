#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import concert_conductor

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':

    rospy.init_node('conductor')
    try:
        conductor = concert_conductor.Conductor()
        conductor.spin()
    except concert_conductor.ConductorFailureException as e:
        rospy.logerr("Conductor : failed to initialise [%s]" % str(e))
