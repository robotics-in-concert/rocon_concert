#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import rospy
import concert_msgs.msg as concert_msgs
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_utilities
#import xmlrpclib

##############################################################################
# Conductor
##############################################################################


class RoleManager(object):
    '''
      Manages connectivity information provided by services and provides this
      for human interactive agent (aka remocon) connections.
    '''
    __slots__ = [
            'roles',
            'publishers',
            'parameters',
            'spin'
        ]

    def __init__(self):
        self.roles = {}
        self.publishers = self._setup_publishers()
        self.parameters = self._setup_parameters()
        self._stub_init()
        # Aliases
        self.spin = rospy.spin

    def _setup_publishers(self):
        publishers = {}
        publishers['icon'] = rospy.Publisher('~icon', rocon_std_msgs.Icon, latch=True)
        icon = rocon_utilities.icon_resource_to_msg('concert_roles/rocon.png')
        publishers['icon'].publish(icon)
        publishers['roles'] = rospy.Publisher('~roles', concert_msgs.Roles, latch=True)
        return publishers

    def _setup_parameters(self):
        param = {}
        #param['config'] = {}
        #param['config']['auto_invite'] = rospy.get_param('~auto_invite', False)
        return param

    def _stub_init(self):
        '''
          Hard codes some roles and apps. Later this configuration should come from
          the services.
        '''
        hard_coded_roles = concert_msgs.Roles()
        hard_coded_roles.list = ['admin', 'dev', 'guzzler']
        self.publishers['roles'].publish(hard_coded_roles)
        for role in hard_coded_roles.list:
            self.roles[role] = []
        android_platform_info_list = [rocon_std_msgs.PlatformInfo.OS_ANDROID,
                                 rocon_std_msgs.PlatformInfo.VERSION_ANY,
                                 rocon_std_msgs.PlatformInfo.PLATFORM_SMART_DEVICE,
                                 rocon_std_msgs.PlatformInfo.SYSTEM_ROSJAVA
                                ]
        android_platform_info = '.'.join(android_platform_info_list)
        icon = rocon_utilities.icon_resource_to_msg('concert_roles/rocon.png')
        self.roles['admin'].append(concert_msgs.RemoconApp(
                                              "com.github.robotics_in_concert.rocon_android.SolutionManager",
                                              android_platform_info,
                                              "Solution Manager",
                                              "Configuration manager for the concert",
                                              icon
                                              )
                                   )
        self.roles['admin'].append(concert_msgs.RemoconApp(
                                              "com.github.robotics_in_concert.rocon_android.SolutionMonitor",
                                              android_platform_info,
                                              "Solution Monitor",
                                              "Monitors various aspects of the solution.",
                                              icon
                                              )
                                   )
        self.roles['dev'].append(concert_msgs.RemoconApp(
                                              "com.github.robotics_in_concert.rocon_android.Pizza",
                                              android_platform_info,
                                              "Pizza Delivery",
                                              "Tantalises and tickles the taste buds with a virtual pizza.",
                                              icon
                                              )
                                   )
        self.roles['dev'].append(concert_msgs.RemoconApp(
                                              "com.github.robotics_in_concert.rocon_android.Beer",
                                              android_platform_info,
                                              "Beer",
                                              "The super sopper.",
                                              icon
                                              )
                                   )
