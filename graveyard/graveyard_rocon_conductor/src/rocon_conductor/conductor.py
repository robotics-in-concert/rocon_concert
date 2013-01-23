'''
Created on 08/08/2011

@author: Daniel Stonier
'''
##############################################################################
# Imports
##############################################################################

import sys

# Ros imports
import roslib
roslib.load_manifest('rocon_conductor')
import rospy

# Local imports
from .connection_manager import Connections
from .zeroconf import listen_for_app_managers

##############################################################################
# Main
##############################################################################


def main():
    rospy.init_node('conductor', log_level=rospy.DEBUG)
    rospy.sleep(1.0)
    if not listen_for_app_managers():
        return 1
    connections = Connections()
    connections.spin()
    return 0

if __name__ == '__main__':
    sys.exit(main())
