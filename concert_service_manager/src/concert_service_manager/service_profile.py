# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import time
import yaml

import concert_msgs.msg as concert_msgs
import genpy
import rocon_python_utils
import rospkg
import rospy
import unique_id
import rocon_console.console as console
import scheduler_msgs.msg as scheduler_msgs

from .exceptions import InvalidServiceProfileException

##############################################################################
# Classes
##############################################################################


class ServiceProfile(object):
    __slots__ = [
            'msg',              # basic data storage for the profile - concert_msgs.ServiceProfile
            '_last_modified',   # timestamp of last .service file modification
            'hash',             # hashlib.sha224 used by the service pool to track this profile
            'resource_name',    # local copy of the resource name
            '_overrides',       # any overrides that need to be applied after any loading
            '_location_cache',  # pointer to the service profile location cache maintained by the service pool { resource_name : filename }
            # aliases
            'name'
        ]

    def __init__(self, hash_id, resource_name, overrides, service_profile_location_cache):
        """
          Loads the service profile given the data provided by the solution configuration
          (i.e. resource name + overrides). We provide an arg for the filename here even
          though we could look it up ourselves since the service manager has probably already
          cached it.

          :param hash_id: hash of the solution configuration data element (for tracking purposes)
          :type hash_id: hashlib.sha224

          :param resource_name: pkg/name identifying a concert service
          :type resource_name: str

          :param overrides: dictionary of the possible overridable variables for a service, key set is bounded
          :type overrides: dic

          :param service_profile_location_cache: this class will use and update the cache if it's own filename changes
          :type service_profile_location_cache: { resource_name : os pathname }

          :returns: the solution configuration data for services in this concert
          :rtype: concert_msgs.ServiceProfile

          :raises: :exc:`concert_service_manager.InvalidServiceProfileException` if the service profile yaml is invalid
        """
        self.hash = hash_id
        self._location_cache = service_profile_location_cache
        self._overrides = overrides
        self.resource_name = resource_name
        self._last_modified = None  # gets updated when we load the profile
        try:
            self.msg = self._load_profile()
            self._last_modified = time.ctime(os.path.getmtime(self._filename()))
        except (rospkg.ResourceNotFound, IOError):
            raise InvalidServiceProfileException("could not find service profile [%s]" % self.resource_name)
        self._validate()  # can also raise InvalidServiceProfileException
        # aliases
        self.name = self.msg.name

    def __str__(self):
        s = ''
        s += console.green + self.hash.hexdigest() + console.reset + '\n'
        s += console.cyan + "     resource_name" + console.reset + ": " + console.yellow + "%s\n" % self.resource_name + console.reset
        s += console.cyan + "     name" + console.reset + ": " + console.yellow + "%s\n" % self.msg.name + console.reset
        s += console.cyan + "     description" + console.reset + ": " + console.yellow + "%s\n" % self.msg.description + console.reset
        s += console.cyan + "     author" + console.reset + ": " + console.yellow + "%s\n" % self.msg.author + console.reset
        s += console.cyan + "     priority" + console.reset + ": " + console.yellow + "%s\n" % self.msg.priority + console.reset
        if self.msg.icon.resource_name:
            s += console.cyan + "     icon" + console.reset + ": " + console.yellow + "%s\n" % self.msg.icon.resource_name + console.reset
        s += console.cyan + "     launcher_type" + console.reset + ": " + console.yellow + "%s\n" % self.msg.launcher_type + console.reset
        if self.msg.launcher:
            s += console.cyan + "     launcher" + console.reset + ": " + console.yellow + "%s\n" % self.msg.launcher + console.reset
        if self.msg.interactions:
            s += console.cyan + "     interactions" + console.reset + ": " + console.yellow + "%s\n" % self.msg.interactions + console.reset
        if self.msg.parameters:
            s += console.cyan + "     parameters" + console.reset + ": " + console.yellow + "%s\n" % self.msg.parameters + console.reset
        return s

    def _validate(self):
        """
        Checks that the service profile is a valid profile.

          :raises: :exc:`concert_service_manager.InvalidServiceProfileException` if the service profile yaml is invalid
        """
        if self.msg.launcher_type != concert_msgs.ServiceProfile.TYPE_ROSLAUNCH and \
           self.msg.launcher_type != concert_msgs.ServiceProfile.TYPE_SHADOW and \
           self.msg.launcher_type != concert_msgs.ServiceProfile.TYPE_CUSTOM:
            raise InvalidServiceProfileException("invalid service launcher type [%s][%s]" % (self.msg.launcher_type, self.resource_name))

    def _load_profile(self):
        """
          :raises: :exc:`rospkg.ResourceNotFound` if the service profile yaml is not found
          :raises: :exc:`concert_service_manager.InvalidServiceProfileException` if the service profile yaml is invalid
        """
        msg = concert_msgs.ServiceProfile()
        try:
            filename = self._filename()  # this can raise ResourceNotFound, IOError
            with open(filename) as f:
                loaded_profile = yaml.load(f)
        except rospkg.ResourceNotFound as e:
            raise e

        loaded_profile['resource_name'] = self.resource_name

        # set priority to default if it was not configured
        if 'priority' not in loaded_profile.keys():
            loaded_profile['priority'] = scheduler_msgs.Request.DEFAULT_PRIORITY

        # override parameters
        for key in loaded_profile:
            if key in self._overrides and self._overrides[key] is not None:
                loaded_profile[key] = self._overrides[key]

        if 'launcher_type' not in loaded_profile.keys():  # not set
            loaded_profile['launcher_type'] = concert_msgs.ServiceProfile.TYPE_SHADOW
        # We need to make sure the service description name is a valid rosgraph namespace name
        # @todo Actually call the rosgraph function to validate this (do they have converters?)
        loaded_profile['name'] = loaded_profile['name'].lower().replace(" ", "_")

        # load icon if necessary
        replace = {}
        if 'icon' in loaded_profile:
            replace[loaded_profile['icon']] = rocon_python_utils.ros.icon_resource_to_msg(loaded_profile['icon'])

        # generate message
        genpy.message.fill_message_args(msg, loaded_profile, replace)

        # fill in unique identifier used by services and their requesters
        msg.uuid = unique_id.toMsg(unique_id.fromRandom())

        return msg

    def reload(self):
        """
          Attempt to reload the profile from file if it has changed.

          :raises: :exc:`concert_service_manager.InvalidServiceProfileException` if the service profile yaml is invalid
        """
        modified_time = time.ctime(os.path.getmtime(self._filename()))
        try:
            if modified_time != self._last_modified:
                self.msg = self._load_profile()
                self._last_modified = time.ctime(os.path.getmtime(self._filename()))
        except (rospkg.ResourceNotFound, IOError):
            raise InvalidServiceProfileException("could not find service profile [%s]" % self.resource_name)

    def _filename(self):
        '''
          Try and find the service profile filename. Update the cache if necessary.

          :raises: :exc:`rospkg.ResourceNotFound` if the service profile yaml is not found
        '''
        # 1st attempt - look in da cache
        try:
            if os.path.isfile(self._location_cache[self.resource_name]):
                return self._location_cache[self.resource_name]
        except KeyError:
            pass
        # 2nd attempt - do the hard yakka
        try:
            filename = rocon_python_utils.ros.find_resource_from_string(self.resource_name)
            self._location_cache[self.resource_name] = filename  # update the cache
            return filename
        except rospkg.ResourceNotFound:
            raise rospkg.ResourceNotFound("could not find service profile [%s]" % self.resource_name)
