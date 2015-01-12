# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import hashlib
import os.path
import time
import yaml

import rospkg
import rospy
import rocon_python_utils
import rocon_console.console as console
import rocon_std_msgs.msg as rocon_std_msgs

from .exceptions import InvalidSolutionConfigurationException
from .exceptions import InvalidServiceProfileException, NoServiceExistsException
from .service_profile import ServiceProfile
from .utils import *

##############################################################################
# Classes
##############################################################################


def load_solution_configuration(yaml_file):
    """
      Load the solution configuration from a yaml file. This is a pretty
      simple file, just a list of services specified by resource names
      along with overrides for that service.

      The overrides can be empty (None) or is an unstructured yaml
      representing configuration of the service that is modified.
      This is not validated against the actual service here.

      :param yaml_file: filename of the solution configuration file
      :type yaml_file: str

      :returns: the solution configuration data for services in this concert
      :rtype: [ServiceData]

      :raises: :exc:`concert_service_manager.InvalidSolutionConfigurationException` if the yaml provides invalid configuration
    """
    service_configurations = []
    # read
    with open(yaml_file) as f:
        service_list = yaml.load(f)
        for s in service_list:
            overrides = s['overrides'] if 'overrides' in s else None
            service_data = ServiceConfigurationData(s['resource_name'], overrides)
            service_configurations.append(service_data)
    # validate
    identifiers = []
    for service_data in service_configurations:
        if service_data.overrides['name']:
            identifier = service_data.overrides['name']
        else:
            identifier = service_data.resource_name
        if identifier in identifiers:
            raise InvalidSolutionConfigurationException("service configuration found with duplicate names [%s]" % identifier)
        else:
            identifiers.append(identifier)
    return service_configurations

##############################################################################
# Classes
##############################################################################


class ServiceConfigurationData(object):

    """
      Represents a single entry in the solution service configuration (.services) file.
      We hash the important bits so we can track when the file changes.
    """

    __slots__ = [
        'resource_name',                       # ros resource name for the service
        'overrides',                            # dictionary of override keys for the service (use a dic so we can pack it into a ros msg later)
        'hash',                                  # hash of this configuration data
        '_is_reading_from_cache'              # flag for checking whether reading chache or not
    ]

    override_keys = ['name', 'description', 'icon', 'priority', 'interactions', 'parameters']

    def __init__(self, resource_name, overrides):
        """
          Process data from the spec into variables.

          :param resource_name: pkg/name identifying a concert service
          :type resource_name: str

          :param overrides: dictionary of the possible overridable variables for a service, key set is bounded
          :type overrides: dic
        """
        # initialisation
        self.resource_name = resource_name
        self.overrides = {}
        for key in ServiceConfigurationData.override_keys:
            self.overrides[key] = overrides[key] if overrides and key in overrides.keys() else None
        # warnings
        if overrides:
            invalid_keys = [key for key in overrides.keys() if key not in ServiceConfigurationData.override_keys]
            for key in invalid_keys:
                rospy.logwarn("Service Manager : invalid key in the service soln configuration yaml [%s]" % key)
        s = resource_name
        for value in self.overrides.values():
            s += str(value)
        self.hash = hashlib.sha224(s)


class ServicePool(object):

    """
      Stores the current solution's service related configuration. This is
      obtained from a ros resource yaml file which typically specifies the
      list of services permitted to run as well as any overridable
      configuration they may have.
    """
    __slots__ = [
        '_yaml_file',      # os.path to solution configuration file
        '_last_modified',  # timestamp of last file modifications
        'service_profiles',  # { name : ServiceProfile }
        '_cached_service_profile_locations'  # all known service profiles on the ros package path : { resource_name : os.path to .service file }
    ]

    def __init__(self, resource_name, concert_name):
        """
          Initialise the class with a pointer to the yaml that will be
          scanned and later monitored for changes that can be applied
          to a running concert.

          :param resource_name: pkg/filename of a yaml formatted service configuration for a solution
          :type resource_name: str
          :param resource_name: concert name for storing cache file
          :type resource_name: str

          :raises: :exc:`rospkg.ResourceNotFound` if resource_name cannot be resolved.
          :raises: :exc:`concert_service_manager.InvalidSolutionConfigurationException` if the yaml provides invalid configuration
        """
        self.service_profiles = {}
        # cache initial service locations to save resource name lookup times.
        cached_service_profile_information, unused_invalid_services = rocon_python_utils.ros.resource_index_from_package_exports(rocon_std_msgs.Strings.TAG_SERVICE)
        self._cached_service_profile_locations = {}
        for cached_resource_name, (cached_filename, unused_catkin_package) in cached_service_profile_information.iteritems():
            self._cached_service_profile_locations[cached_resource_name] = cached_filename
        # init cache path
        concert_name = concert_name.lower().replace(' ', '_')
        setup_home_dirs(concert_name)
        # load
        if resource_name != "":
            try:
                (self._is_reading_from_cache, self._yaml_file) = self._check_cache(rocon_python_utils.ros.find_resource_from_string(resource_name), concert_name)  # rospkg.ResourceNotFound
                service_configurations = load_solution_configuration(self._yaml_file)  # InvalidSolutionConfigurationException
                self._load_service_profiles(service_configurations)
                self._save_yaml_cache(concert_name)
                self._last_modified = time.ctime(os.path.getmtime(self._yaml_file))
            except (rospkg.ResourceNotFound, InvalidSolutionConfigurationException) as e:
                raise e

    def _save_yaml_cache(self, concert_name):
        """
          Save yaml cache file in temporary directory. If service manager is restarted, it load cached yaml file.

          :param yaml_file: temporary location for storing cache file as concert name
          :type str
        """

        yaml_file_name = self._yaml_file.split('/')[-1]
        if '.services' in yaml_file_name:
            yaml_stream = yaml.load(open(self._yaml_file, 'r'))
            cache_yaml_file = get_home(concert_name) + '/' + yaml_file_name
            cache_yaml_stream = file(cache_yaml_file, 'w')
            yaml.safe_dump(yaml_stream, cache_yaml_stream, default_flow_style=False)

    def _check_cache(self, yaml_file, concert_name):
        """
          Check whether cached yaml file is existed or not

          :param yaml_file: yaml file name about services list for checking
          :type str
          :param yaml_file: temporary location for checking as concert name
          :type str

          :returns: flag and full path yaml file as result of check existence about cache file. If cache file is existed, return true and cache path. Otherwise, return false and original path
          :rtype: (bool, str)

        """

        yaml_file_name = yaml_file.split('/')[-1]
        cache_yaml_file = get_home(concert_name) + '/' + yaml_file_name
        if os.path.isfile(cache_yaml_file):
            return (True, cache_yaml_file)
        else:
            return (False, yaml_file)

    def __str__(self):
        s = ''
        s += console.bold + 'Service Profiles: \n' + console.reset
        for service_profile in self.service_profiles.values():
            s += " - %s" % service_profile
        return s

    def __len__(self):
        return len(self.service_profiles)

    def _load_service_profiles(self, service_configurations):
        """
          Generate profiles from the specified configurations and load into the
          service pool, give warnings if it happens to abort. Don't raise exceptions
          here, just gracefully ignore.

          :param service_configurations: configurations to generate profiles from
          :type service_configurations: [ServiceConfigurationData]
        """
        for service_configuration in service_configurations:
            try:
                service_profile = ServiceProfile(
                    service_configuration.hash,
                    service_configuration.resource_name,
                    service_configuration.overrides,
                    self._cached_service_profile_locations,
                    self._is_reading_from_cache
                )
            except (InvalidServiceProfileException) as e:
                rospy.logwarn("Service Manager : %s" % e)
                continue
            if service_profile.msg.name in self.service_profiles.keys():
                rospy.logwarn("Service Manager : tried to load a duplicate service name [%s][%s]" % (service_profile.resource_name, service_profile.msg.name))
            else:
                self.service_profiles[service_profile.msg.name] = service_profile

    def reload(self):
        """
          Check the timestamp of the file and reload it if necessary. If you need locking, make sure you
          call this inside a locked scope from the outside.

          :raises: :exc:`concert_service_manager.InvalidSolutionConfigurationException` if the yaml provides invalid configuration
        """
        modified_time = time.ctime(os.path.getmtime(self._yaml_file))
        try:
            if modified_time != self._last_modified:
                service_configurations = load_solution_configuration(self._yaml_file)  # generates InvalidSolutionConfigurationException
                hashes = [s.hash.digest() for s in service_configurations]
                new_configurations = [s for s in service_configurations if self._find_by_hash(s.hash) is None]
                lost_profiles = [s for s in self.service_profiles.values() if s.hash.digest() not in hashes]
                #unchanged_profiles = [s for s in self.service_profiles.values() if s not in lost_profiles]
                self._load_service_profiles(new_configurations)
                for profile in lost_profiles:
                    del profile
                self._last_modified = modified_time
        except InvalidSolutionConfigurationException as e:
            raise e

    def _find_by_hash(self, hash_id):
        """
          Scan the service pool looking for profiles with the specified hash. This is
          an internal convenience function.

          :returns: the service configuration for a match or None if not found
          :rtype: ServiceConfigurationData or None
        """
        for service_profile in self.service_profiles.values():
            if service_profile.hash.digest() == hash_id.digest():
                return service_profile
        return None

    def find(self, name):
        """
          Scan the service data to see if a service has been configured with the specified name. Check if it
          has changed internally and reload it if necessary before returning it.

          :param name: name of the service profile to find
          :type name: str

          :returns: the service profile that matches
          :rtype: ServiceProfile

          :raises: :exc:`concert_service_manager.NoServiceExistsException` if the service profile is not available
        """
        try:
            service_profile = self.service_profiles[name]
            # make sure it's up to date
            # this will be negligible cost if the service profile file did not change
            service_profile.reload()
            # check if the name changed
            if service_profile.name != name:
                rospy.logwarn("Service Manager : we are in the shits, the service profile name changed %s->%s" % (name, service_profile.name))
                rospy.logwarn("Service Manager : TODO upgrade the find function with a full reloader")
                raise NoServiceExistsException("we are in the shits, the service profile name changed %s->%s" % (name, service_profile.name))
            return service_profile
        except KeyError:
            raise NoServiceExistsException("service profile not found in the configured service pool [%s]" % name)
