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
import std_msgs.msg as std_msgs
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_std_msgs.srv as rocon_std_srvs
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
            'concert_name',
            'roles',
            'publishers',
            'parameters',
            'services',
            'spin',
            'platform_info'
        ]

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self):
        self.concert_name = rospy.get_param('~concert_name', 'concert1')
        self.roles = {}
        self.publishers = self._setup_publishers()
        self.services = self._setup_services()
        self.parameters = self._setup_parameters()
        self.platform_info = rocon_std_msgs.PlatformInfo(
                                os=rocon_std_msgs.PlatformInfo.OS_LINUX,
                                version=rocon_std_msgs.PlatformInfo.VERSION_ANY,
                                platform=rocon_std_msgs.PlatformInfo.PLATFORM_PC,
                                system=rocon_std_msgs.PlatformInfo.SYSTEM_ROS,
                                name=self.concert_name)
        self._stub_init()
        # Aliases
        self.spin = rospy.spin

    def _setup_publishers(self):
        publishers = {}
        publishers['info']  = rospy.Publisher('~info', rocon_std_msgs.PlatformInfo, latch=True)
        publishers['roles'] = rospy.Publisher('~roles', concert_msgs.Roles, latch=True)

        return publishers

    def _setup_services(self):
        '''
          These are all public services. Typically that will drop them into the /concert
          namespace.
        '''
        services = {}
        services['get_roles_and_apps'] = rospy.Service('get_roles_and_apps',
                                                       concert_srvs.GetRolesAndApps,
                                                       self._ros_service_filter_roles_and_apps)
        services['set_roles_and_apps'] = rospy.Service('set_roles_and_apps',
                                                       concert_srvs.SetRolesAndApps,
                                                       self._ros_service_filter_roles_and_apps)
        services['request_interaction'] = rospy.Service('request_interaction',
                                                       concert_srvs.SetRolesAndApps,
                                                       self._ros_service_filter_roles_and_apps)
        services['get_platform_info'] = rospy.Service('~get_platform_info',
                                                      rocon_std_srvs.GetPlatformInfo,
                                                      self._ros_service_get_platform_info)
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
        self.publishers['info'].publish(self.platform_info)
        self.publishers['roles'].publish(hard_coded_roles)

        for role in hard_coded_roles.list:
            self.roles[role] = []
        platform_info = rocon_std_msgs.PlatformInfo(
                                    os=rocon_std_msgs.PlatformInfo.OS_ANDROID,
                                    version=rocon_std_msgs.PlatformInfo.VERSION_ANY,
                                    platform=rocon_std_msgs.PlatformInfo.PLATFORM_SMART_DEVICE,
                                    system=rocon_std_msgs.PlatformInfo.SYSTEM_ROSJAVA,
                                    name='*',  # Not relevant to this list
                                    )
        self.roles['Admin'].append(RemoconApp(
                                              name="com.github.robotics_in_concert.rocon_android.SolutionManager",
                                              platform_info=platform_info,
                                              display_name="Solution Manager",
                                              description="Configuration manager for the concert",
                                              service_name="admin",
                                              remappings=[],
                                              parameters=[],
                                              )
                                   )
        self.roles['Admin'].append(RemoconApp(
                                              name="com.github.robotics_in_concert.rocon_android.SolutionMonitor",
                                              platform_info=platform_info,
                                              display_name="Solution Monitor",
                                              description="Monitors various aspects of the solution.",
                                              service_name="admin",
                                              remappings=[],
                                              parameters=[]
                                              )
                                   )
        self.roles['Dev'].append(RemoconApp(
                                              name="com.github.robotics_in_concert.rocon_android.Pizza",
                                              platform_info=platform_info,
                                              display_name="Pizza Delivery",
                                              description="Tantalises and tickles the taste buds with a virtual pizza.",
                                              service_name="spooning",
                                              remappings=[],
                                              parameters=[rocon_std_msgs.KeyValue('type', 'super supreme')]
                                              )
                                   )
        self.roles['Guzzler'].append(RemoconApp(
                                              name="com.github.robotics_in_concert.rocon_android.Beer",
                                              platform_info=platform_info,
                                              display_name="Beer",
                                              description="The super sopper.",
                                              service_name="leeching",
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
        print 'roles_and_apps service result: ',roles_and_apps
        return roles_and_apps

    def _ros_service_get_platform_info(self, request):
        '''
          Handle incoming requests to provide platform info, same way robots do.
        '''
        platform_info = rocon_std_srvs.GetPlatformInfoResponse()
        platform_info.platform_info = self.platform_info
        return platform_info
