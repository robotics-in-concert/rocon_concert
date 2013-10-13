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
import concert_msgs.srv as concert_srvs
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_utilities
from .remocon_app import RemoconApp
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
            'services',
            'spin'
        ]

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self):
        self.roles = {}
        self.publishers = self._setup_publishers()
        self.services = self._setup_services()
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

    def _setup_services(self):
        services = {}
        services['get_roles_and_apps'] = rospy.Service('~get_roles_and_apps',
                                                       concert_srvs.GetRolesAndApps,
                                                       self._ros_service_filter_roles_and_apps)
        return services

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
        hard_coded_roles.list = ['Admin', 'Dev', 'Guzzler']
        self.publishers['roles'].publish(hard_coded_roles)
        for role in hard_coded_roles.list:
            self.roles[role] = []
        icon = rocon_utilities.icon_resource_to_msg('concert_roles/rocon.png')
        platform_info = rocon_std_msgs.PlatformInfo(
                            os=rocon_std_msgs.PlatformInfo.OS_ANDROID,
                            version=rocon_std_msgs.PlatformInfo.VERSION_ANY,
                            platform=rocon_std_msgs.PlatformInfo.PLATFORM_SMART_DEVICE,
                            system=rocon_std_msgs.PlatformInfo.SYSTEM_ROSJAVA,
                            name='*',  # Not relevant to this list
                            icon=icon
                            )
        self.roles['Admin'].append(RemoconApp(
                                              name="com.github.robotics_in_concert.rocon_android.SolutionManager",
                                              platform_info=platform_info,
                                              display_name="Solution Manager",
                                              description="Configuration manager for the concert",
                                              remappings=[],
                                              parameters=[],
                                              )
                                   )
        self.roles['Admin'].append(RemoconApp(
                                              name="com.github.robotics_in_concert.rocon_android.SolutionMonitor",
                                              platform_info=platform_info,
                                              display_name="Solution Monitor",
                                              description="Monitors various aspects of the solution.",
                                              remappings=[],
                                              parameters=[]
                                              )
                                   )
        self.roles['Dev'].append(RemoconApp(
                                              name="com.github.robotics_in_concert.rocon_android.Pizza",
                                              platform_info=platform_info,
                                              display_name="Pizza Delivery",
                                              description="Tantalises and tickles the taste buds with a virtual pizza.",
                                              remappings=[],
                                              parameters=[rocon_std_msgs.KeyValue('type', 'super supreme')]
                                              )
                                   )
        self.roles['Guzzler'].append(RemoconApp(
                                              name="com.github.robotics_in_concert.rocon_android.Beer",
                                              platform_info=platform_info,
                                              display_name="Beer",
                                              description="The super sopper.",
                                              remappings=[rocon_std_msgs.Remapping('/fosters', '/beer/leffe')],
                                              parameters=[]
                                              )
                                    )

    ##########################################################################
    # Ros Service Callbacks
    ##########################################################################

    def _ros_service_filter_roles_and_apps(self, request):
        '''
          Handle incoming requests to provide a role-applist dictionary
          filtered for the requesting platform.
        '''
        roles_and_apps = concert_srvs.GetRolesAndAppsResponse()
        roles_and_apps.data = []
        if request.roles:  # works for None or empty list
            # Validate that each role is in self.roles here
            roles = request.roles
        else:
            roles = self.roles.keys()
        for role in roles:
            role_app_list = concert_msgs.RoleAppList(role, [])
            for remocon_app in self.roles[role]:
                if remocon_app.is_runnable(request.platform_info):
                    role_app_list.remocon_apps.append(remocon_app.to_msg())
            roles_and_apps.data.append(role_app_list)
        return roles_and_apps
