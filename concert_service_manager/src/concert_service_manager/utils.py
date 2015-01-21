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

##############################################################################
# Methods
##############################################################################


def get_concert_home(concert_name):
    '''
      Retrieve the location of the home directory for the service manager
      temporary storage needs as concert name.
      If it is not existed, create new one and return this.

      @return the service manager home directory (path object).
      @type str
    '''
    SOLUTION_ROOT = 'solutions'
    concert_home = os.path.join(rocon_python_utils.ros.get_rocon_home(), SOLUTION_ROOT, concert_name)
    if not os.path.isdir(concert_home):
        os.makedirs(concert_home)
    return concert_home


def get_service_profile_cache_home(concert_name, service_name):
    '''
      Retrieve the location of the directory used for storing service profile.
      If it is not existed, create new one and return this.

      @return the rocon remocon qt settings directory (path object).
      @type str
    '''

    service_profile_cache_home = os.path.join(get_concert_home(concert_name), 'services', service_name)
    if not os.path.isdir(service_profile_cache_home):
        os.makedirs(service_profile_cache_home)
    return service_profile_cache_home
