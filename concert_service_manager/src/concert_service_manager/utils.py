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
        os.makedirs(get_home(concert_name))


def setup_service_profile_home_dirs(concert_name, service_name):
    '''
      Retrieve the location of the home directory for the service manager
      temporary storage needs as concert name

      @return the service manager home directory (path object).
      @type str
    '''
    if not os.path.isdir(get_service_profile_cache_home(concert_name, service_name)):
        os.makedirs(get_service_profile_cache_home(concert_name, service_name))


def get_home(concert_name):
    '''
      Retrieve the location of the home directory for the service manager
      temporary storage needs as concert name

      @return the service manager home directory (path object).
      @type str
    '''
    return os.path.join(rospkg.get_ros_home(), 'rocon', concert_name)


def get_service_profile_cache_home(concert_name, service_name):
    '''
      Retrieve the location of the directory used for storing service configuration.

      @return the rocon remocon qt settings directory (path object).
      @type str
    '''
    return os.path.join(get_home(concert_name), 'services', service_name)


def check_extension_name(file_name, extension_name):
    '''
    Check whether file name include extension name. If it does not include extension name, return file name added extension name.

    @return file name included extension name
    @type str
    '''

    if extension_name in file_name:
        return file_name
    else:
        return file_name + extension_name
