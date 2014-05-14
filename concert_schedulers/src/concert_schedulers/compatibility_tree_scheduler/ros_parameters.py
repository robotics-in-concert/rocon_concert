#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
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
    '''
    param = {}
    param['debug_show_compatibility_tree'] = rospy.get_param('~debug_show_compatibility_tree', True)
    param['enable_preemptions'] = rospy.get_param('~enable_preemptions', True)

    return param
