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
from .exceptions import InvalidSolutionConfigurationException
from .exceptions import InvalidServiceProfileException, NoServiceExistsException
from .utils import *

##############################################################################
# Classes
##############################################################################


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
      :param is_cached: todo
      :type is_cached: bool

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
                    rospy.logwarn("Service Manager : invalid key in the service soln configuration yaml [%s]" % key)
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


class ServiceCacheManager(object):

    __slots__ = [
        '_concert_name',            # todo
        '_resource_name',           # todo
        '_cache_service_list',     # todo
        '_modification_callback',  # todo
        'service_profiles',         # todo
        'msg',                        # todo
    ]

    def __init__(self, concert_name, resource_name, modification_callback=None):
        self._concert_name = concert_name.strip().lower().replace(' ', '_')
        self._resource_name = resource_name
        self._modification_callback = modification_callback
        self._cache_service_list = {}
        self.service_profiles = {}

        setup_home_dirs(self._concert_name)
        self.load_service_cache()

    def _init_service_cache_list(self):
        """
          Todo

        """
        self._cache_service_list = self._get_service_cache_list()

    def _get_service_cache_list(self):
        """
          Todo

          :returns: cache list
          :rtype: dict

        """
        cache_list = {}
        for root, dirs, files in os.walk(get_home(self._concert_name)):
            for dir_name in dirs:
                dir_path = str(os.path.join(root, dir_name))
                cache_list[dir_path] = {}
                cache_list[dir_path]['type'] = 'directory'
                cache_list[dir_path]['last_modified_time'] = time.ctime(os.path.getmtime(dir_path))

            for file_name in files:
                file_path = str(os.path.join(root, file_name))
                cache_list[file_path] = {}
                cache_list[file_path]['type'] = 'file'
                cache_list[file_path]['last_modified_time'] = time.ctime(os.path.getmtime(file_path))
        return cache_list

    def _check_service_cache(self):
        """
          Check whether cached yaml file regarding service profile is generated or not

          :returns: flag and full path yaml file as result of check existence about cache file. If cache file is existed, return true and cache path. Otherwise, return false and original path
          :rtype: str

        """
        check_result = True
        default_service_configuration_file = rocon_python_utils.ros.find_resource_from_string(self._resource_name)
        service_configuration_file_name = default_service_configuration_file.split('/')[-1]
        service_configuration_file = get_home(self._concert_name) + '/' + service_configuration_file_name
        if not os.path.isfile(service_configuration_file) or os.stat(service_configuration_file).st_size <= 0:
            check_result = False
            self._loginfo("load from default: [%s]" % self._resource_name)
        else:
            self._loginfo("load from cache: [%s]" % service_configuration_file)
        return (check_result, service_configuration_file)

    def _create_service_cache(self):
        """
          Create cache as loading service configuration from default value.

        """
        # read resource file
        loaded_profiles = {}
        service_configurations = load_solution_configuration_from_default(rocon_python_utils.ros.find_resource_from_string(self._resource_name))
        for service_configuration in service_configurations:
            resource_name = check_extension_name(service_configuration['resource_name'], '.service')
            overrides = service_configuration['overrides']

            loaded_profile = self._load_service_profile_from_default(resource_name, overrides)
            self._save_service_profile(loaded_profile)
            loaded_profiles[loaded_profile['name']] = copy.deepcopy(loaded_profile)
        # save solution configuration
        self._save_solution_configuration(loaded_profiles)

    def _load_service_profile_from_default(self, resource_name, overrides):
        """
          Todo: Create cache as loading service configuration from default value.

          :param resource_name: Todo
          :type resource_name: str
          :param overrides: Todo
          :type overrides: dict

          :return: dictionary about service profile
          :rtype: dict

          :raises: :exc:`rospkg.ResourceNotFound` Todo

        """
        # load service profile from default
        try:
            file_name = rocon_python_utils.ros.find_resource_from_string(resource_name)
            with open(file_name) as f:
                loaded_profile = yaml.load(f)
        except rospkg.ResourceNotFound as e:
            raise e

        loaded_profile['resource_name'] = resource_name
        # set priority to default if it was not configured
        if 'priority' not in loaded_profile.keys():
            loaded_profile['priority'] = scheduler_msgs.Request.DEFAULT_PRIORITY
        for key in loaded_profile:
            if key in overrides and overrides[key] is not None:
                loaded_profile[key] = overrides[key]
        if 'launcher_type' not in loaded_profile.keys():  # not set
            loaded_profile['launcher_type'] = concert_msgs.ServiceProfile.TYPE_SHADOW
        loaded_profile['name'] = loaded_profile['name'].strip().lower().replace(" ", "_")

        if 'parameters' in loaded_profile.keys():
            loaded_profile['parameters_detail'] = []
            # todo if cache, if original
            parameters_yaml_file = rocon_python_utils.ros.find_resource_from_string(check_extension_name(loaded_profile['parameters'], '.parameters'))
            try:
                with open(parameters_yaml_file) as f:
                    parameters_yaml = yaml.load(f)
                    loaded_profile['parameters_detail'] = parameters_yaml
            except rospkg.ResourceNotFound as e:
                raise e

        if 'interactions' in loaded_profile.keys():
            interactions_yaml_file = rocon_python_utils.ros.find_resource_from_string(check_extension_name(loaded_profile['interactions'], '.interactions'))
            try:
                with open(interactions_yaml_file) as f:
                    interactions_yaml = yaml.load(f)
                    loaded_profile['interactions_detail'] = interactions_yaml
            except rospkg.ResourceNotFound as e:
                raise e

        return loaded_profile

    def _load_service_cache_from_cache(self, services_file_name):
        '''
        Todo

        :param service_file_name: todo
        :type service_file_name: str

        :raises: :exc:`rospkg.ResourceNotFound` Todo

        '''
        with open(services_file_name) as f:
            service_list = yaml.load(f)
        for service in service_list:
            try:
                service_file_name = os.path.join(get_service_profile_cache_home(self._concert_name, service['name']), check_extension_name(service['name'], '.service'))
                if not os.path.isfile(service_file_name) or os.stat(service_file_name).st_size <= 0:
                    rospy.logwarn("Service Manager : can not find service file in cache [%s]" % check_extension_name(service['name'], '.service'))
                    continue
                with open(service_file_name) as f:
                    loaded_profile = yaml.load(f)
                    if 'parameters' in loaded_profile.keys():
                        loaded_profile['parameters_detail'] = []
                        parameters_yaml_file = os.path.join(get_service_profile_cache_home(self._concert_name, service['name']), loaded_profile['parameters'])
                        try:
                            with open(parameters_yaml_file) as f:
                                parameters_yaml = yaml.load(f)
                                loaded_profile['parameters_detail'] = parameters_yaml
                        except rospkg.ResourceNotFound as e:
                            raise e

                    if 'interactions' in loaded_profile.keys():
                        loaded_profile['interactions_detail'] = []
                        interactions_yaml_file = os.path.join(get_service_profile_cache_home(self._concert_name, service['name']), loaded_profile['interactions'])
                        try:
                            with open(interactions_yaml_file) as f:
                                interactions_yaml = yaml.load(f)
                                loaded_profile['interactions_detail'] = interactions_yaml
                        except rospkg.ResourceNotFound as e:
                            raise e

            except rospkg.ResourceNotFound as e:
                raise e

            loaded_profile['msg'] = self._service_profile_to_msg(loaded_profile)
            self.service_profiles[loaded_profile['name']] = copy.deepcopy(loaded_profile)

    def _service_profile_to_msg(self, loaded_profile):
        '''
        Todo

        :returns: generated service profile message
        :rtype: [concert_msgs.ServiceProfile]

        '''

        msg = concert_msgs.ServiceProfile()
        msg.uuid = unique_id.toMsg(unique_id.fromRandom())
        # todo change more nice method
        if 'resource_name' in loaded_profile:
            msg.resource_name = loaded_profile['resource_name']
        if 'name' in loaded_profile:
            msg.name = loaded_profile['name']
        if 'description' in loaded_profile:
            msg.description = loaded_profile['description']
        if 'author' in loaded_profile:
            msg.author = loaded_profile['author']
        if 'priority' in loaded_profile:
            msg.priority = loaded_profile['priority']
        if 'launcher_type' in loaded_profile:
            msg.launcher_type = loaded_profile['launcher_type']
        if 'icon' in loaded_profile:
            msg.icon = rocon_python_utils.ros.icon_resource_to_msg(loaded_profile['icon'])
        if 'launcher' in loaded_profile:
            msg.launcher = loaded_profile['launcher']
        if 'interactions' in loaded_profile:
            msg.interactions = loaded_profile['interactions']
        if 'parameters' in loaded_profile:
            msg.parameters = loaded_profile['parameters']
        if 'parameters_detail' in loaded_profile:
            for param_key in loaded_profile['parameters_detail'].keys():
                msg.parameters_detail.append(rocon_std_msgs.KeyValue(param_key, loaded_profile['parameters_detail'][param_key]))

        return msg

    def _save_service_profile(self, loaded_service_profile_from_file):
        '''
        Todo

        :param loaded_service_profile_from_file: Todo
        :type loaded_service_profile_from_file: dict

        '''
        loaded_profile = copy.deepcopy(loaded_service_profile_from_file)
        service_name = loaded_profile['name']

        setup_service_profile_home_dirs(self._concert_name, service_name)
        service_profile_cache_home = get_service_profile_cache_home(self._concert_name, service_name)

        # writting interaction data
        if 'interactions_detail' in loaded_profile.keys():
            service_interactions_file_name = os.path.join(service_profile_cache_home, check_extension_name(service_name, '.interactions'))
            loaded_profile['interactions'] = service_interactions_file_name.split('/')[-1]
            with file(service_interactions_file_name, 'w') as f:
                yaml.safe_dump(loaded_profile['interactions_detail'], f, default_flow_style=False)
            del (loaded_profile['interactions_detail'])

        # writting parameter data
        if 'parameters_detail' in loaded_profile.keys():
            service_parameters_file_name = os.path.join(service_profile_cache_home, check_extension_name(service_name, '.parameters'))
            loaded_profile['parameters'] = service_parameters_file_name.split('/')[-1]
            with file(service_parameters_file_name, 'w') as f:
                yaml.safe_dump(loaded_profile['parameters_detail'], f, default_flow_style=False)
            del (loaded_profile['parameters_detail'])

        # if 'launcher' in loaded_profile.keys():
        #    loaded_profile['launcher'] = rocon_python_utils.ros.find_resource_from_string(loaded_profile['launcher'] + '.launch')

        # delete msg data
        if 'msg' in loaded_profile.keys():
            del (loaded_profile['msg'])

        # writting service profile data
        service_profile_file_name = os.path.join(service_profile_cache_home, check_extension_name(service_name, '.service'))
        with file(service_profile_file_name, 'w') as f:
            yaml.safe_dump(loaded_profile, f, default_flow_style=False)

    def _save_solution_configuration(self, service_profiles):
        '''
        Todo

        :param service_profiles: Todo
        :type service_profiles: dict

        '''
        solution_configuration = []
        for service_profile in service_profiles.keys():
            configuration_item = {}
            configuration_item['name'] = service_profile
            configuration_item['enabled'] = False
            solution_configuration.append(configuration_item)

        service_configuration_file_name = rocon_python_utils.ros.find_resource_from_string(self._resource_name).split('/')[-1]
        if '.services' in service_configuration_file_name:
            cache_srv_config_file = get_home(self._concert_name) + '/' + service_configuration_file_name
            with file(cache_srv_config_file, 'w') as f:
                yaml.safe_dump(solution_configuration, f, default_flow_style=False)

    def _find_resource_from_string(self, resource):
        '''
        Todo, It is not implemented,yet.

        :param resource: Todo
        :type resource: string

        '''
        # todo package name cahce
        package, filename = roslib.names.package_resource_name(resource)
        if not package:
            raise rospkg.ResourceNotFound("resource could not be split with a valid leading package name [%s]" % (resource))
        try:
            resolved = roslib.packages.find_resource(package, filename, None)
            if not resolved:
                raise rospkg.ResourceNotFound("cannot locate [%s] in package [%s]" % (filename, package))
            elif len(resolved) == 1:
                return resolved[0]
            elif len(resolved) > 1:
                raise rospkg.ResourceNotFound("multiple resources named [%s] in package [%s]:%s\nPlease specify full path instead" % (filename, package, ''.join(['\n- %s' % r for r in resolved])))
        except rospkg.ResourceNotFound:
            raise rospkg.ResourceNotFound("[%s] is not a package or launch file name [%s]" % (package, package + '/' + filename))
        return None

    def _loginfo(self, msg):
        rospy.loginfo("Service Manager : " + str(msg))

    def _logwarn(self, msg):
        rospy.logwarn("Service Manager : " + str(msg))

    def update_service_cache(self, service_profile_msg):
        '''
        Todo 

        :param service_profile_msg: Todo
        :type service_profile_msg: concert_msgs/ServiceProfile

        :return: boolean result of update cache with message
        :rtype: (bool, str)
        '''

        result = True
        message = ""

        service_profile = message_converter.convert_ros_message_to_dictionary(service_profile_msg)
        service_name = service_profile['name']
        service_parameter_detail = {}
        for param_pair in service_profile['parameters_detail']:
            service_parameter_detail[param_pair['key']] = param_pair['value']

        if service_profile['name'] in self.service_profiles.keys():
            service_profile = self.service_profiles[service_profile['name']]
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

    def load_service_cache(self):
        '''
        Todo

        '''
        (check_result, solution_configuration_cache) = self._check_service_cache()
        if not check_result:
            self._create_service_cache()
        self._load_service_cache_from_cache(solution_configuration_cache)
        self._init_service_cache_list()

    def check_modification_service_cache(self):
        '''
        Todo

        '''
        if self._cache_service_list != self._get_service_cache_list():
            self.load_service_cache()
            if self._modification_callback:
                self._modification_callback()

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
            if service_profile['name'] != name:
                rospy.logwarn("Service Manager : we are in the shits, the service profile name changed %s->%s" % (name, service_profile['name']))
                rospy.logwarn("Service Manager : TODO upgrade the find function with a full reloader")
                raise NoServiceExistsException("we are in the shits, the service profile name changed %s->%s" % (name, service_profile['name']))
            return service_profile
        except KeyError:
            raise NoServiceExistsException("service profile not found in the configured service pool [%s]" % name)
