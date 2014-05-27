#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
"""
.. module:: ros_parameters

This module reads parameters relevant to the concert conductor from the
ros parameter server.
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
      Currently this looks for the following parameters:

      * ~auto_invite (false) : don't automatically invite clients
      * ~local_clients_only (false) : don't invite clients from other pc's on the network, used for simulations.
      * ~oblivian_timeout (3600) : time before a bad, gone client is removed from the index.

      :returns: dictionary of parameters
      :rtype: dict { parameter name : value }
    '''
    param = {}
    param['auto_invite'] = rospy.get_param('~auto_invite', False)
    param['local_clients_only'] = rospy.get_param('~local_clients_only', False)
    param['oblivion_timeout'] = rospy.get_param('~oblivion_timeout', 3600)
    return param
