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
import copy

import rospkg
import roslib.names
import rospy
import unique_id
import rocon_python_utils
import rocon_std_msgs.msg as rocon_std_msgs
import concert_msgs.msg as concert_msgs
import scheduler_msgs.msg as scheduler_msgs

from rospy_message_converter import message_converter
from .service_profile import ServiceProfile
from .exceptions import InvalidSolutionConfigurationException
from .exceptions import InvalidServiceProfileException, NoServiceExistsException
from .utils import *


def load_solution_configuration_from_default(yaml_file):
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
    override_keys = ['name', 'description', 'icon', 'priority', 'interactions', 'parameters']
    with open(yaml_file) as f:
        service_list = yaml.load(f)
        for s in service_list:
            service_data = {}
            service_data['resource_name'] = s['resource_name']
            loaded_overrides = s['overrides'] if 'overrides' in s else None
            overrides = {}
            for key in override_keys:
                overrides[key] = loaded_overrides[key] if loaded_overrides and key in loaded_overrides.keys() else None
            # warnings
            if loaded_overrides:
                invalid_keys = [key for key in loaded_overrides.keys() if key not in override_keys]
                for key in invalid_keys:
                    rospy._logwarn("invalid key in the service soln configuration yaml [%s]" % key)
            service_data['overrides'] = copy.deepcopy(overrides)
            service_configurations.append(service_data)

    # validate
    identifiers = []
    for service_data in service_configurations:
        if service_data['overrides']['name'] is not None and 'name' in service_data['overrides'].keys():
            identifier = service_data['overrides']['name']
        else:
            identifier = service_data['resource_name']

        if identifier in identifiers:
            raise InvalidSolutionConfigurationException("service configuration found with duplicate names [%s]" % identifier)
        else:
            identifiers.append(identifier)
    return service_configurations

##############################################################################
# Classes
##############################################################################


class ServicePool(object):

    __slots__ = [
        '_concert_name',                # concert name to classify cache directory
        '_resource_name',               # service resource name. It is composed 'package name/service resource name'. ex.)chatter_concert/chatter.service
        '_modification_callback',     # callback function. It is called when service_caches_mod_time is changed.
        '_disable_cache',               # flag regarding whether using cache
        'service_profiles',            # dictionary of service profile to use in service manager. {'service name':ServiceProfile class}
        '_solution_config_mtime',     # lastest modification time of loaded solution configuration file
        '_solution_config_file',      # full path of loaded solution configuration file
    ]

    def __init__(self, concert_name, resource_name, disable_cache, modification_callback=None):
        self._concert_name = rocon_python_utils.ros.get_ros_friendly_name(concert_name)
        self._resource_name = resource_name
        self._disable_cache = disable_cache
        self._modification_callback = modification_callback
        self._solution_config_mtime = None
        self._solution_config_file = ''
        self.service_profiles = {}
        self._load_services()

    def _load_services(self):
        """
        Load services from solution configuration loaded cached or default.
        """
        try:
            service_config_file = ''
            if self._disable_cache:
                self._loginfo("load service profile from default configuration")
                service_config_file = self._load_service_profiles_from_default()
            else:
                self._loginfo("load service profile from cached configuration")
                service_config_file = self._load_service_profiles_from_cache()
            self._solution_config_file = service_config_file
            self._solution_config_mtime = time.ctime(os.path.getmtime(service_config_file))

        except rospkg.ResourceNotFound as e:
            self._logwarn(str(e))

    def _load_service_profiles_from_default(self):
        """
        Load service profile from default.

        :returns: todo
        :rtype: str

        """
        default_service_config_file = rocon_python_utils.ros.find_resource_from_string(self._resource_name)
        loaded_solution_config = load_solution_configuration_from_default(default_service_config_file)
        for service in loaded_solution_config:
            service_profile_file = rocon_python_utils.ros.check_extension_name(service['resource_name'], '.service')
            overrides = service['overrides']
            try:
                read_profile = ServiceProfile(concert_name=self._concert_name,
                                              is_read_from_default=True,
                                              service_profile_file=service_profile_file,
                                              overrides=overrides)
                self.service_profiles[read_profile.name] = read_profile
            except rospkg.ResourceNotFound as e:
                self._logwarn('cannot load service profile: [%s][%s]' % (service_profile_file,e))
                continue
        return default_service_config_file

    def _load_service_profiles_from_cache(self):
        """
        Load service profile from cached solution configuration. If cahed file does not existe, load it from default

        :returns: file path of solution configuration 
        :rtype: str

        """
        (check_result, cached_solution_config_file) = self._check_cache()
        if not check_result:
            self._loginfo("load from default configuration to create cache: [%s]" % self._resource_name)
            self._load_service_profiles_from_default()
            self._loginfo("create cache file: [%s]" % cached_solution_config_file)
            self._create_cache()

        with open(cached_solution_config_file) as f:
            service_list = yaml.load(f)
            for service in service_list:
                name = service['name']
                enabled = service['enabled']
                service_profile_file = os.path.join(get_service_profile_cache_home(self._concert_name, service['name']), rocon_python_utils.ros.check_extension_name(service['name'], '.service'))
                try:
                    read_profile = ServiceProfile(concert_name=self._concert_name,
                                                  is_read_from_default=False,
                                                  service_profile_file=service_profile_file,
                                                  overrides=None,
                                                  enabled=enabled)
                    self.service_profiles[read_profile.name] = read_profile
                except rospkg.ResourceNotFound as e:
                    self._logwarn('cannot load service profile: [%s]' % service_profile_file)
                    continue

        return cached_solution_config_file

    def _check_cache(self):
        """
          Check whether cached yaml files regarding service profile and solution config are already generated or not

          :returns: flag and full path of cache solution config file or default solution config. If cache file existed, return true and cache path. Otherwise, return false and default resource path
          :rtype: str

          :raises: :exc:`rospkg.ResourceNotFound` 

        """
        is_cached_solution_config = True
        try:
            default_solution_configuration_file = rocon_python_utils.ros.find_resource_from_string(self._resource_name)
        except rospkg.ResourceNotFound as e:
            raise e
        solution_configuration_file_name = default_solution_configuration_file.split('/')[-1]
        cached_solution_configuration_file = get_concert_home(self._concert_name) + '/' + solution_configuration_file_name
        if rocon_python_utils.ros.is_validation_file(cached_solution_configuration_file):
            with open(cached_solution_configuration_file) as f:
                service_list = yaml.load(f)
                for service in service_list:
                    service_file_name = os.path.join(get_service_profile_cache_home(self._concert_name, service['name']), rocon_python_utils.ros.check_extension_name(service['name'], '.service'))
                    if not rocon_python_utils.ros.is_validation_file(service_file_name):
                        is_cached_solution_config = False
                        self._logwarn("Broken service profile files!!")
                        break
                    else:
                        is_cached_solution_config = True
        else:
            is_cached_solution_config = False
            self._logwarn("No cached solution config file!!")

        return (is_cached_solution_config, cached_solution_configuration_file)

    def _create_cache(self):
        """
          Create cache with loaded service configuration from default value.

        """
        # save each loaded service profile
        for sp in self.service_profiles.values():
            self._save_service_profile(sp.service_profile)
        # save solution configuration
        self._save_solution_config()

    def _check_solution_config_modification(self):
        """
        Check modification of solution configuration file. It it is removed and modified, return true.

        :returns: flag whether it is modified or not. 
        :rtype: bool 
        """
        is_changed = False
        if not rocon_python_utils.ros.is_validation_file(self._solution_config_file):
            is_changed = True
        else:
            mtime = time.ctime(os.path.getmtime(self._solution_config_file))
            if self._solution_config_mtime != mtime:
                is_changed = True
                self._solution_config_mtime = mtime
        return is_changed

    def _save_solution_config(self):
        """
        Save solution configuration about currently loaded service profiles

        """
        solution_config = {}
        for sp in self.service_profiles.values():
            solution_config[sp.name] = {'name': sp.name, 'enabled': sp.enabled}
        # write solution config file
        default_solution_config_file = rocon_python_utils.ros.find_resource_from_string(self._resource_name).split('/')[-1]
        cache_solution_config_file = get_concert_home(self._concert_name) + '/' + default_solution_config_file
        with file(cache_solution_config_file, 'w') as f:
            yaml.safe_dump(solution_config.values(), f, default_flow_style=False)
        self._solution_config_file = cache_solution_config_file
        self._solution_config_mtime = time.ctime(os.path.getmtime(cache_solution_config_file))

    def _save_service_profile(self, loaded_service_profile_from_file):
        """
        Save cache about the loaded service profile

        :param loaded_service_profile_from_file: data of dictionary type regarding service profile
        :type loaded_service_profile_from_file: dict

        """
        loaded_profile = copy.deepcopy(loaded_service_profile_from_file)
        service_name = loaded_profile['name']
        service_profile_cache_home = get_service_profile_cache_home(self._concert_name, service_name)

        # writting interaction data
        if 'interactions_detail' in loaded_profile.keys():
            service_interactions_file_name = os.path.join(service_profile_cache_home, rocon_python_utils.ros.check_extension_name(service_name, '.interactions'))
            loaded_profile['interactions'] = service_interactions_file_name.split('/')[-1]
            with file(service_interactions_file_name, 'w') as f:
                yaml.safe_dump(loaded_profile['interactions_detail'], f, default_flow_style=False)
            del (loaded_profile['interactions_detail'])

        # writting parameter data
        if 'parameters_detail' in loaded_profile.keys():
            service_parameters_file_name = os.path.join(service_profile_cache_home, rocon_python_utils.ros.check_extension_name(service_name, '.parameters'))
            loaded_profile['parameters'] = service_parameters_file_name.split('/')[-1]
            with file(service_parameters_file_name, 'w') as f:
                yaml.safe_dump(loaded_profile['parameters_detail'], f, default_flow_style=False)
            del (loaded_profile['parameters_detail'])

        # delete msg key
        if 'msg' in loaded_profile.keys():
            del (loaded_profile['msg'])

        # writting service profile data
        service_profile_file_name = os.path.join(service_profile_cache_home, rocon_python_utils.ros.check_extension_name(service_name, '.service'))
        with file(service_profile_file_name, 'w') as f:
            yaml.safe_dump(loaded_profile, f, default_flow_style=False)

    def _loginfo(self, msg):
        rospy.loginfo("Service Manager : " + str(msg))

    def _logwarn(self, msg):
        rospy.logwarn("Service Manager : " + str(msg))

    def reload_services(self):
        """
        Reload services as checking modification of solution congfiguration and service profile.
        If they are changed, service reloaded and modification callback is called.

        """
        # check solution config
        if self._check_solution_config_modification():
            self._loginfo('detect changed solution config. reload')
            self._load_services()
            if self._modification_callback:
                self._modification_callback()
        # check service profile
        for sp in self.service_profiles.values():
            if sp._check_modification():
                try:
                    sp.reload()
                except (rospkg.ResourceNotFound, IOError) as e:
                    self._loginfo('[%s] service profile is broken.' % sp.name)
                    self._load_services()
                if self._modification_callback:
                    self._modification_callback()

    def get_solution_config(self):
        """
        Get current solution configuration status.

        :returns: dictionary of solution configuration. It is made up service name and enabled status 
        :rtype: dict

        """
        solution_config = {}
        for sp in self.service_profiles.values():
            solution_config[sp.name] = {'name': sp.name, 'enabled': sp.enabled}
        return solution_config

    def find(self, name):
        """
          Scan the service data to see if a service has been configured with the specified name. Check if it
          has changed internally and reload it if necessary before returning it.

          :param name: name of the service profile to find
          :type name: str

          :returns: the service profile that matches
          :rtype: dict

          :raises: :exc:`concert_service_manager.NoServiceExistsException` if the service profile is not available
        """
        try:
            service_profile = self.service_profiles[name]
            # check if the name changed
            if service_profile.name != name:
                self._logwarn("we are in the shits, the service profile name changed %s->%s" % (name, service_profile['name']))
                self._logwarn("TODO upgrade the find function with a full reloader")
                raise NoServiceExistsException("we are in the shits, the service profile name changed %s->%s" % (name, service_profile['name']))
            return service_profile
        except KeyError:
            raise NoServiceExistsException("service profile not found in the configured service pool [%s]" % name)

    def update_service_cache(self, service_profile_msg):
        """
        Update service cache with ros message regarding service profile

        :param service_profile_msg: ros message of service profile. concert_msgs/ServiceProfile
        :type service_profile_msg: concert_msgs/ServiceProfile

        :return: boolean result of update cache with message
        :rtype: (bool, str)
        """
        result = True
        message = ""
        if self._disable_cache:
            result = False
            message = "'disable_cache' option is true.\n If you want to chage parameter. Set the 'disable_cache' option into false"
        else:
            service_profile = message_converter.convert_ros_message_to_dictionary(service_profile_msg)
            service_name = service_profile['name']
            service_parameter_detail = {}
            for param_pair in service_profile['parameters_detail']:
                service_parameter_detail[param_pair['key']] = param_pair['value']

            if service_profile['name'] in self.service_profiles.keys():
                service_profile = self.service_profiles[service_profile['name']].service_profile
                service_profile['parameters_detail'] = service_parameter_detail
                try:
                    self._save_service_profile(service_profile)
                    result = True
                    message = 'Success'
                except:
                    result = False
                    message = "Fail during saveing service profile"
            else:
                result = False
                message = "Can not find service: %s" % service_name
        return (result, message)

    def update_solution_configuration(self, service_profiles_msg):
        """
        Update service enabled status in cached solution config file. It is just update when "disable_cache" option is false.

        :param service_profiles_msg: updated data of dictionary type regarding service profile
        :type service_profiles_msg: concert_msgs/ServiceProfile
        """
        if not self._disable_cache:
            is_change = False
            for service_profile_msg in service_profiles_msg:
                service_profile = message_converter.convert_ros_message_to_dictionary(service_profile_msg)
                service_name = service_profile['name']
                service_enabled = service_profile['enabled']
                if service_enabled != self.service_profiles[service_name].enabled:
                    self.service_profiles[service_name].enabled = service_enabled
                    is_change = True
            if is_change:
                self._save_solution_config()
