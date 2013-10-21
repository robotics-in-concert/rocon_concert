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
#import rocon_utilities
#import xmlrpclib

##############################################################################
# Conductor
##############################################################################


class RemoconApp(object):
    '''
      All the glorious details for a remocon app. Some of this will be
      constant and should later be obtained from a repository. Some of
      this may be overridable from service/solution configuration.
    '''
    __slots__ = [
            'name',
            'platform_info',
            'display_name',
            'description',
            'service_name',
            'remappings',
            'parameters'
        ]

    def __init__(self, 
                 name="unknown_app.py",
                 platform_info=rocon_std_msgs.PlatformInfo(),
                 display_name="Unknown App",
                 description="Unknown description.",
                 service_name="self-serving service",
                 remappings=[],
                 parameters=[]
                 ):
        self.name = name
        self.platform_info = platform_info
        self.display_name = display_name
        self.description = description
        self.service_name = service_name
        self.remappings = remappings
        self.parameters = parameters

    def is_runnable(self, remocon_platform_info):
        '''
          Checks the given remocon platform info and makes sure this app is compatible.

          @param remocon_platform_info : remocon platform information.
          @type concert_msgs.PlatformInfo

          @return true if compatible, false otherwise
          @rtype Bool
        '''
        if remocon_platform_info is None:
            return True
        if remocon_platform_info.os == '' or remocon_platform_info.os == '*':
            return True
        elif remocon_platform_info.os != self.platform_info.os:
            return False
        # Not worrying about version check yet (should need it for android soon)
        # Not worrying about platform check yet
        # Not worrying about system check yet
        # Not worrying about name check yet
        return True

    def to_msg(self):
        '''
          Converts the appropriate information in this class into a message format that
          is consumed by remocons to configure their app lists.

          @return appropriate remocon app data so the remocon can configure and launch the app.
          @rtype concert_msgs.RemoconApp
        '''
        return concert_msgs.RemoconApp(
                                name=self.name,
                                display_name=self.display_name,
                                description=self.description,
                                service_name=self.service_name,
                                icon=self.platform_info.icon,
                                remappings=self.remappings,
                                parameters=self.parameters
                                )
