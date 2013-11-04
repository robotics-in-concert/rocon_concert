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
import rocon_utilities

##############################################################################
# Conductor
##############################################################################


class ConcertMaster(object):
    __slots__ = [
            'publishers',
            'param',
            'spin'
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
        self.publishers['info'].publish(concert_info)
        # Aliases
        self.spin = rospy.spin

    def _setup_ros_parameters(self):
        '''
          These parameters are all public parameters - subsequently they'll usually be
          found under the /concert namespace.
        '''
        param = {}
        param['name'] = rospy.get_param('name', 'Concert')
        param['icon'] = rospy.get_param('icon', 'concert_master/rocon_logo.png')
        param['description'] = rospy.get_param('description', 'A rocon concert.')
        return param
