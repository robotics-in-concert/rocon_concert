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

import rocon_console.console as console
import rospy

###############################################################################
# Functions
###############################################################################


class Parameters:
    """
    The variables of this class are default constructed from parameters on the
    ros parameter server. Each parameter is nested in the private namespace of
    the node which instantiates this class.

    :ivar oblivian_timeout: time before a bad, gone client is removed from the index. *[3600]*
    :vartype oblivian_timeout: str

    .. _rocon_launch: http://wiki.ros.org/rocon_launch
    .. _rocon_uri: http://wiki.ros.org/rocon_uri
    .. _resource name: http://wiki.ros.org/Names#Package_Resource_Names
    """
    def __init__(self):
        # see sphinx docs above for more detailed explanations of each parameter
        self.oblivion_timeout = rospy.get_param('~oblivion_timeout', 3600)

    def __str__(self):
        s = console.bold + "\nConductor Parameters:\n" + console.reset
        for key in sorted(self.__dict__):
            s += console.cyan + "    %s: " % key + console.yellow + "%s\n" % (self.__dict__[key] if self.__dict__[key] is not None else '-')
        s += console.reset
        return s
