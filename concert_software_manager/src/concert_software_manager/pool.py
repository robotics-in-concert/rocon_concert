# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import yaml
import os

import genpy
import rocon_python_utils
import rocon_std_msgs.msg as rocon_std_msgs
import concert_msgs.msg as concert_msgs

from .exceptions import InvalidSoftwareprofileException

##############################################################################
# Classes
##############################################################################

class SoftwareProfile(object):
    '''
    parses software profile from file path. 
    '''
    def __init__(self, resource_name, filepath):
        self.resource_name = resource_name
        self._filepath = filepath
        try:
            self._load_profile()
        except (rospkg.ResourceNotFound, IOError) as e:
            raise InvalidSoftwareProfileException("couldn't parse software profile[%s]"%(str(e)))
            
    def _load_profile(self):
        '''
        load profile from filepath
        '''
        msg  = concert_msgs.SoftwareProfile()

        with open(self._filename) as f:
            loaded_profile = yaml.load(f)

        loaded_profile['resource_name'] = self.resource_name
        loaded_profile['name'] = loaded_profile['name'].lower().replace(" ", "_")

        # generate message
        genpy.message.fill_message_args(msg, loaded_profile)
        self.msg = msg 
    def to_msg(self):
        return self.msg

    def __str__(self):
        s = ''
        s += console.cyan + "     resource_name" + console.reset + ": " + console.yellow + "%s\n" % self.resource_name + console.reset
        s += console.cyan + "     name" + console.reset + ": " + console.yellow + "%s\n" % self.msg.name + console.reset
        s += console.cyan + "     description" + console.reset + ": " + console.yellow + "%s\n" % self.msg.description + console.reset
        s += console.cyan + "     author" + console.reset + ": " + console.yellow + "%s\n" % self.msg.author + console.reset
        s += console.cyan + "     launch" + console.reset + ": " + console.yellow + "%s\n" % self.msg.launch + console.reset
        return s


class SoftwarePool(object):
    '''
      maintains scanned software in ros package path.
    '''
    __slots__ = ['_registered_software', '_software_profiles', '_invalid_software_profiles']

    def __init__(self):
        self._registered_software = self._scan_registered_software()
        self._software_profiles, self._invalid_software_profiles = self._load_software_profiles(self._registered_software)

    def status(self):
        return self._software_profiles, invalid_software_profiles

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
        
        :param software_locations: file path to the software profile
        :type software_locations: {resourcen_name: filepath} 

        :returns: loaded software profile
        :rtypes: { resource_name: SoftwareProfile}
        '''
        profiles = {}
        invalid_profiles = {}
        for name, location in software_locations.items():
            try:
                profiles[name] = SoftwareProfile(name, location) 
            except InvalidSoftwareprofileException as e:
                invalid_profiles[name] = str(e)
        return profiles
