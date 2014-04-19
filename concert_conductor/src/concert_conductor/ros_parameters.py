#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
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
    param['auto_invite'] = rospy.get_param('~auto_invite', False)
    param['local_clients_only'] = rospy.get_param('~local_clients_only', False)
    param['oblivion_timeout'] = rospy.get_param('~oblivion_timeout', 3600)  # time before a bad, gone client is removed from the index.
    return param
