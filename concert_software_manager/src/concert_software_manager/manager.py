# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import concert_msgs.msg as concert_msgs
from .pool import SoftwarePool

##############################################################################
# Manager
##############################################################################

class Manager(object):

    def __init__(self):
        self._params = self._setup_ros_parameters()
        self._software_pool = SoftwarePool()

    def _setup_ros_parameters(self):
        params = {}

    def spin(self):
        rospy.spin()

    def loginfo(self, msg):
        rospy.loginfo('Software Farm : %s'%str(msg))

    def logwarn(self, msg):
        rospy.logwarn('Software Farm : %s'%str(msg))
