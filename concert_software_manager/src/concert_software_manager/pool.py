# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rocon_python_utils
import rocon_std_msgs.msg as rocon_std_msgs
import concert_msgs.msg as concert_msgs
import yaml
import os

##############################################################################
# Classes
##############################################################################

class SoftwareProfile(object):
    '''
    parses software profile from file path 
    '''
    def __init__(self, resource_name, filepath):
        self.resource_name = resource_name
        self._filepath = filepath
        self._load_profile()

    def _load_profile(self):
        '''
        load profile from filepath
        '''
        msg  = concert_msgs.SoftwareProfile()

        with open(self._filename) as f:
            loaded_profile = yaml.load(f)

        loaded_profile['resource_name'] = self.resource_name

        

class SoftwarePool(object):
    '''
      maintains scanned software in ros package path.
    '''

    def __init__(self):
        self._registered_software = self._scan_registered_software()
        self._software_profiles, invalid_software_profiles = self._load_software_profiles(self._registered_software)
    
    def _scan_registered_software(self): 
        '''
        parses package exports to scan all available software in the system. and returns name and location
        '''
        cached_software_profile_information, unused_invalid_software = rocon_python_utils.ros.resource_index_from_package_exports(rocon_std_msgs.Strings.TAG_SOFTWARE)
        cached_service_profile_locations = {}
        for cached_resource_name, (cached_filename, unused_catkin_package) in cached_service_profile_information.iteritems():
            cached_service_profile_locations[cached_resource_name] = cached_filename

        return cached_service_profile_locations

    def _load_software_profiles(self, software_locations):
        '''
        Load software profile from file path 
        '''
        profiles = {}
        for name, location in software_locations.items():
            profiles[name] = SoftwareProfile(name, location) 
