# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import time
import yaml
import copy
import json

import genpy
import rospkg
import rospy
import unique_id

import rocon_python_utils
import scheduler_msgs.msg as scheduler_msgs
import rocon_std_msgs.msg as rocon_std_msgs
import concert_msgs.msg as concert_msgs
from .utils import *

##############################################################################
# Classes
##############################################################################


class ServiceProfile(object):
    __slots__ = [
        '_concert_name',  # concert name for loading service profile from cache
        '_profile_files',  # a list of pair about service profile files and lasted modification time (*.service, *.parameters, *.interactions)
        '_is_read_from_default',  # flag whether service profile read from default or not
        '_service_profile_file',  # file path containing service description; "*.service" file path
        '_overrides',  # any overrides that need to be applied after any loading
        'service_profile',   # data storage for the profile as type of dictionary
        'name',  # service name
        'msg',  # basic data storage for the profile - concert_msgs.ServiceProfile
        'enabled',  # enabled status
    ]

    def __init__(self, concert_name, is_read_from_default, service_profile_file, overrides=[], enabled=None):
        self._concert_name = concert_name
        self._is_read_from_default = is_read_from_default
        self.service_profile = None
        self._profile_files = []

        self._service_profile_file = service_profile_file
        self._overrides = overrides

        loaded_profile = {}
        try:
            if self._is_read_from_default:
                loaded_profile = self._read_service_profile_from_default()
            else:
                loaded_profile = self._read_service_profiles_from_cache()
        except (rospkg.ResourceNotFound, IOError) as e:
            raise e

        self.service_profile = copy.deepcopy(loaded_profile)
        self.name = self.service_profile['name']
        self.msg = self._service_profile_to_msg(self.service_profile)
        self.enabled = enabled

    def _check_modification(self):
        """
        Check modification of service profile files.

        :return: result of modification checking. If they are removed or modified, return true.
        :rtype: bool 
        """
        is_change = False
        for profile_file in self._profile_files:
            file_name = profile_file[0]
            modified_time = profile_file[1]
            if not rocon_python_utils.ros.is_validation_file(file_name):
                is_change = True
            elif modified_time != time.ctime(os.path.getmtime(file_name)):
                is_change = True
                profile_file[1] = time.ctime(os.path.getmtime(file_name))
        return is_change

    def reload(self):
        """
        Reload the service profile from cached or default file as flag. 
        However, enabled status of service is not update.

        :raises: :exc:`rospkg.ResourceNotFound` if the service profile is not available
        """
        self._loginfo("detect [%s] service modification. reload" % self.name)
        loaded_profile = {}
        try:
            if self._is_read_from_default:
                loaded_profile = self._read_service_profile_from_default()
            else:
                loaded_profile = self._read_service_profiles_from_cache()
        except rospkg.ResourceNotFound as e:
            raise e

        self.service_profile = copy.deepcopy(loaded_profile)
        self.name = self.service_profile['name']
        self.msg = self._service_profile_to_msg(self.service_profile)

    def _service_profile_to_msg(self, loaded_profile):
        """
        Change service proflies data to ros message

        :returns: generated service profile message
        :rtype: [concert_msgs.ServiceProfile]

        """
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
                msg.parameters_detail.append(rocon_std_msgs.KeyValue(param_key, str(loaded_profile['parameters_detail'][param_key])))
                # t = type(loaded_profile['parameters_detail'][param_key])
                # if t is list or t is dict:
                #     msg.parameters_detail.append(rocon_std_msgs.KeyValue(param_key, eval(loaded_profile['parameters_detail'][param_key])))
                # else:
                #     msg.parameters_detail.append(rocon_std_msgs.KeyValue(param_key, str(loaded_profile['parameters_detail'][param_key])))
        return msg

    def _read_service_profile_from_default(self):
        """
          Load service profile from resource file.

          :returns: file path of loaded service profile
          :rtype: str

          :raises: :exc:`rospkg.ResourceNotFound` 

        """
        # load service profile from resource
        loaded_profile = {}
        service_file_name = self._service_profile_file
        overrides = copy.deepcopy(self._overrides)

        try:
            file_name = rocon_python_utils.ros.find_resource_from_string(service_file_name)
            with open(file_name) as f:
                loaded_profile = yaml.load(f)
            self._profile_files.append([file_name, time.ctime(os.path.getmtime(file_name))])
        except rospkg.ResourceNotFound as e:
            raise e
        loaded_profile['resource_name'] = service_file_name

        # set priority to default if it was not configured
        if 'priority' not in loaded_profile.keys():
            loaded_profile['priority'] = scheduler_msgs.Request.DEFAULT_PRIORITY
        for key in loaded_profile:
            if key in overrides and overrides[key] is not None:
                loaded_profile[key] = overrides[key]
        if 'launcher_type' not in loaded_profile.keys():  # not set
            loaded_profile['launcher_type'] = concert_msgs.ServiceProfile.TYPE_SHADOW
        loaded_profile['name'] = rocon_python_utils.ros.get_ros_friendly_name(loaded_profile['name'])

        if 'parameters' in loaded_profile.keys():
            loaded_profile['parameters_detail'] = []
            try:
                parameters_yaml_file = rocon_python_utils.ros.find_resource_from_string(rocon_python_utils.ros.check_extension_name(loaded_profile['parameters'], '.parameters'))
                with open(parameters_yaml_file) as f:
                    parameters_yaml = yaml.load(f)
                    loaded_profile['parameters_detail'] = parameters_yaml
                self._profile_files.append([parameters_yaml_file, time.ctime(os.path.getmtime(parameters_yaml_file))])
            except rospkg.ResourceNotFound as e:
                raise e

        if 'interactions' in loaded_profile.keys():
            try:
                interactions_yaml_file = rocon_python_utils.ros.find_resource_from_string(rocon_python_utils.ros.check_extension_name(loaded_profile['interactions'], '.interactions'))
                with open(interactions_yaml_file) as f:
                    interactions_yaml = yaml.load(f)
                    loaded_profile['interactions_detail'] = interactions_yaml
                self._profile_files.append([interactions_yaml_file, time.ctime(os.path.getmtime(interactions_yaml_file))])
            except rospkg.ResourceNotFound as e:
                raise e

        return loaded_profile

    def _read_service_profiles_from_cache(self):
        """
        Load service profile from cached solution configuration.

        :returns: file path of loaded service profile
        :rtype: str

        :raises: :exc:`rospkg.ResourceNotFound`

        """
        loaded_profile = {}
        concert_name = self._concert_name
        service_file_name = self._service_profile_file

        if not rocon_python_utils.ros.is_validation_file(service_file_name):
            raise rospkg.ResourceNotFound("can not find service file in cache [%s]" % service_file_name)
        else:
            self._profile_files.append([service_file_name, time.ctime(os.path.getmtime(service_file_name))])
            with open(service_file_name) as f:
                loaded_profile = yaml.load(f)
                if 'parameters' in loaded_profile.keys():
                    loaded_profile['parameters_detail'] = []
                    parameters_yaml_file = os.path.join(get_service_profile_cache_home(concert_name, loaded_profile['name']), loaded_profile['parameters'])
                    if not rocon_python_utils.ros.is_validation_file(parameters_yaml_file):
                        raise rospkg.ResourceNotFound("can not find parameters file in cache [%s]" % parameters_yaml_file)
                    self._profile_files.append([parameters_yaml_file, time.ctime(os.path.getmtime(parameters_yaml_file))])
                    with open(parameters_yaml_file) as f:
                        parameters_yaml = yaml.load(f)
                        loaded_profile['parameters_detail'] = parameters_yaml
                if 'interactions' in loaded_profile.keys():
                    loaded_profile['interactions_detail'] = []
                    interactions_yaml_file = os.path.join(get_service_profile_cache_home(concert_name, loaded_profile['name']), loaded_profile['interactions'])
                    if not rocon_python_utils.ros.is_validation_file(interactions_yaml_file):
                        raise rospkg.ResourceNotFound("can not find interactions file in cache [%s]" % interactions_yaml_file)
                    self._profile_files.append([interactions_yaml_file, time.ctime(os.path.getmtime(interactions_yaml_file))])
                    with open(interactions_yaml_file) as f:
                        interactions_yaml = yaml.load(f)
                        loaded_profile['interactions_detail'] = interactions_yaml
        return loaded_profile

    def _loginfo(self, msg):
        rospy.loginfo("Service Manager : " + str(msg))

    def _logwarn(self, msg):
        rospy.logwarn("Service Manager : " + str(msg))
