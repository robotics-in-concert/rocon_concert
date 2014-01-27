#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import concert_msgs.msg as concert_msgs
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_utilities

##############################################################################
# Conductor
##############################################################################


class ConcertMaster(object):
    __slots__ = [
            'publishers',
            'param',
            'spin',
        ]

    def __init__(self):
        ##################################
        # Pubs, Subs and Services
        ##################################
        self.publishers = {}
        # efficient latched publisher, put in the public concert namespace.
        self.param = self._setup_ros_parameters()
        self.publishers["info"] = rospy.Publisher("info", concert_msgs.ConcertInfo, latch=True)
        concert_info = concert_msgs.ConcertInfo()
        concert_info.name = self.param['name']
        concert_info.description = self.param['description']
        concert_info.icon = rocon_utilities.icon_resource_to_msg(self.param['icon'])
        concert_info.version = rocon_std_msgs.Strings.ROCON_VERSION
        self.publishers['info'].publish(concert_info)
        # Aliases
        self.spin = rospy.spin

    def _setup_ros_parameters(self):
        '''
          Parameters that are configurable (over-ridable) are currently set via args in the
          concert master launcher where they are published as parameters. We grab those here.

          Parameters that are fixed (not configurable), we set here so we can access the message
          string constant and use that (also to avoid roslaunch clutter).
        '''
        param = {}
        param['name'] = rospy.get_param('name', 'Concert')
        param['icon'] = rospy.get_param('icon', 'concert_master/rocon_logo.png')
        param['description'] = rospy.get_param('description', 'A rocon concert.')
        rospy.set_param('version', rocon_std_msgs.Strings.ROCON_VERSION)
        return param
