# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import yaml
import os

import rospkg
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

        with open(self._filepath) as f:
            loaded_profile = yaml.load(f)

        if not 'launch' in loaded_profile:
            raise InvalidSoftwareprofileException("'launch' field does not exist!!!")
        msg.resource_name = loaded_profile['resource_name'] = self.resource_name
        msg.name          = loaded_profile['name'] = loaded_profile['name'].lower().replace(" ", "_")
        msg.description   = loaded_profile['description'] if 'description' in  loaded_profile else ""
        msg.author        = loaded_profile['author'] if 'author' in loaded_profile else ""
        msg.max_count     = loaded_profile['max_count'] if 'max_count' in loaded_profile else -1
        msg.launch        = loaded_profile['launch'] 
        msg.parameters    = []
        if 'parameters' in loaded_profile:
            for kv in loaded_profile['parameters']:
                msg.parameters.append(rocon_std_msgs.KeyValue(str(kv['name']), str(kv['value'])))

        # generate message
        self.msg = msg 
        self.name = msg.name
    def to_msg(self):
        return self.msg

    def __str__(self):
        s = ''
        s += "\tresource_name : %s\n" % self.resource_name
        s += "\tname          : %s\n" % self.msg.name
        s += "\tdescription   : %s\n" % self.msg.description
        s += "\tauthor        : %s\n" % self.msg.author
        s += "\tlaunch        : %s\n" % self.msg.launch
        s += "\tparameters:       \n"
        for p in self.msg.parameters:
            s += "\t\t%s : %s\n"%(p.key,p.value)

        return s

    def validate_parameters(self, given):
        '''
          validates the given parameter is usable for this software.

          :param given rocon_std_msgs/KeyValue[]: parameter to validate

          :returns: whether it is successful or not, the updated parameters, msg
          :rtypes: bool, [rocon_std_msgs.KeyValue], str
        '''
        default = self.msg.parameters
        default_dict = { i.key:i.value for i in default}
        given_dict = {i.key:i.value for i in given}

        # Check if given parameters are invalid
        d_keys = default_dict.keys()
        is_invalid = False
        invalid_params = []
        for k in given_dict.keys():
            if not k in d_keys:
                invalid_params.append(k)
                is_invalid = True

        if is_invalid:
            msg = "Invalid parameter is given. %s"%str(invalid_params)
            return False, [], msg

        # Assign
        params = {}
        for key, value in default_dict.items():
            params[key] = given_dict[key] if key in given_dict else value
        params_keyvalue = [rocon_std_msgs.KeyValue(key, value) for key, value in params.items()]
        return True, params_keyvalue, "Success"


class SoftwarePool(object):
    '''
      maintains scanned software in ros package path.
    '''
    __slots__ = ['_registered_software', '_software_profiles', '_invalid_software_profiles']

    def __init__(self):
        self._registered_software = self._scan_registered_software()
        self._software_profiles, self._invalid_software_profiles = self._load_software_profiles(self._registered_software)

    def status(self):
        '''
          Returns the current software pool status

          :returns: Avaialble Software profiles, Invalid software profiles
          :rtype: dict, dict
        '''
        return self._software_profiles, self._invalid_software_profiles

    def get_profile(self, resource_name):
        '''
          Returns the profile of given resource name

          :returns: Software profile object 
          :rtype: :exc:`SoftwareProfile`

          :raises: :exc:`SoftwareNotExistException` if the software profile is not available in pool
        '''
        if not resource_name in self._software_profiles: 
            raise SoftwareNotExistException("%s does not exist"%str(resource_name))
        return self._software_profiles[resource_name]


    def _scan_registered_software(self): 
        '''
        parses package exports to scan all available software in the system. and returns name and location
        '''
        cached_software_profile_information, unused_invalid_software = rocon_python_utils.ros.resource_index_from_package_exports(rocon_std_msgs.Strings.TAG_SOFTWARE)
        cached_software_profile_locations = {}
        for cached_resource_name, (cached_filename, unused_catkin_package) in cached_software_profile_information.iteritems():
            cached_software_profile_locations[cached_resource_name] = cached_filename

        return cached_software_profile_locations

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
        return profiles, invalid_profiles

