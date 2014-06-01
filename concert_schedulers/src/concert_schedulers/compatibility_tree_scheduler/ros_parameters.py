#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
"""
.. module:: compatibility_tree_scheduler.ros_parameters

Ros parameter parsing for the scheduler.
"""
##############################################################################
# Imports
##############################################################################

import rospy

###############################################################################
# Functions
###############################################################################


def setup_ros_parameters():
    '''
      Returns validated parameters for this module from the ros param server.

      :returns: parameter dictionary
      :rtype dict:
    '''
    param = {}
    param['debug_show_compatibility_tree'] = rospy.get_param('~debug_show_compatibility_tree', True)
    param['enable_preemptions'] = rospy.get_param('~enable_preemptions', True)

    return param
