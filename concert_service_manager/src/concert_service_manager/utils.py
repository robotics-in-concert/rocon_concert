#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import rocon_python_utils
import rospkg
import rocon_std_msgs.msg as rocon_std_msgs

##############################################################################
# Methods
##############################################################################


def setup_home_dirs(concert_name):
    if not os.path.isdir(get_home(concert_name)):
        print 'create path: ' + str(get_home(concert_name))
        os.makedirs(get_home(concert_name))


def setup_service_home_dirs(concert_name, service_name):
    if not os.path.isdir(get_service_icon_cache_home(concert_name, service_name)):
        os.makedirs(get_service_icon_cache_home(concert_name, service_name))
    if not os.path.isdir(get_service_config_cache_home(concert_name, service_name)):
        os.makedirs(get_service_config_cache_home(concert_name, service_name))


def get_home(concert_name):
    '''
      Retrieve the location of the home directory for the service manager
      temporary storage needs as concert name

      @return the service manager home directory (path object).
      @type str
    '''
    return os.path.join(rospkg.get_ros_home(), 'rocon', concert_name)


def get_service_icon_cache_home(concert_name, service_name):
    '''
      Retrieve the location of the directory used for storing icons about service.

      @return the rocon remocon icons directory (path object).
      @type str
    '''
    return os.path.join(get_home(concert_name), 'icons')


def get_service_config_cache_home(concert_name, service_name):
    '''
      Retrieve the location of the directory used for storing service configuration.

      @return the rocon remocon qt settings directory (path object).
      @type str
    '''
    return os.path.join(get_home(concert_name), 'service_name')
