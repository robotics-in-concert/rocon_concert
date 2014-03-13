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

    """
#      :raises: :exc:`concert_service_manager.InvalidSolutionConfigurationException` if the yaml provides invalid configuration
    service_configurations = {}
    # read
    with open(yaml_file) as f:
        service_list = yaml.load(f)
        for s in service_list:
            overrides = s['overrides'] if 'overrides' in s else None
            service_data = ServiceConfigurationData(s['resource_name'], overrides)
            service_configurations[service_data.id] = service_data
    # validate
#     identifiers = []
#     for service_data in services:
#         if service_data.overrides['name']:
#             identifier = service_data.overrides['name']
#         else:
#             identifier = service_data.resource_name
#         if identifier in identifiers:
#             raise InvalidSolutionConfigurationException("service configuration found with duplicate names [%s]" % identifier)
#         else:
#             identifiers.append(identifier)
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
            'resource_name',  # ros resource name for the service
            'overrides',      # dictionary of override keys for the service (use a dic so we can pack it into a ros msg later)
            'id',             # hash of this configuration data
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
        self.id = hashlib.sha224(s)

    def __str__(self):
        s = ''
        s += console.green + self.id.hexdigest() + console.reset + '\n'
        s += console.cyan + "     resource_name" + console.reset + ": " + console.yellow + "%s\n" % self.resource_name + console.reset
        for key in ServiceConfigurationData.override_keys:
            if self.overrides[key] is not None:
                s += console.cyan + "     " + key + console.reset + ": " + console.yellow + "%s" % self.overrides[key] + console.reset + '\n'
        return s


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
            'available_services',  # [ServiceProfile]
            '_known_services'  # all known services on the ros package path : { resource_name : os.path to .service file }
        ]

    def __init__(self, resource_name):
        """
          Initialise the class with a pointer to the yaml that will be
          scanned and later monitored for changes that can be applied
          to a running concert.

          :param resource_name: pkg/filename of a yaml formatted service configuration for a solution
          :type resource_name: str

          :raises: :exc:`rospkg.ResourceNotFound` if resource_name cannot be resolved.
          :raises: :exc:`concert_service_manager.InvalidSolutionConfigurationException` if the yaml provides invalid configuration
        """
        self._known_services, unused_invalid_services = rocon_python_utils.ros.resource_index_from_package_exports(rocon_std_msgs.Strings.TAG_SERVICE)
        try:
            self._yaml_file = rocon_python_utils.ros.find_resource_from_string(resource_name)
        except rospkg.ResourceNotFound as e:
            raise e
        self._last_modified = time.ctime(os.path.getmtime(self._yaml_file))
        try:
            self.available_services = load_solution_configuration(self._yaml_file)
        except InvalidSolutionConfigurationException as e:
            raise e

    def __str__(self):
        s = ''
        s += console.bold + 'Known Services: \n\n' + console.reset
        for known_service in self._known_services.keys():
            s += console.green + " - " + str(known_service) + "\n" + console.reset
        s += "\n"
        s += console.bold + 'Configured Services: \n\n' + console.reset
        for service in self.available_services.values():
            s += " - %s" % service
        return s

    def __len__(self):
        return len(self.available_services)

    def reload(self):
        """
          Check the timestamp of the file and reload it if necessary.

          :raises: :exc:`concert_service_manager.InvalidSolutionConfigurationException` if the yaml provides invalid configuration
        """
        modified_time = time.ctime(os.path.getmtime(self._yaml_file))
        try:
            if modified_time != self._last_modified:
                self.available_services = load_solution_configuration(self._yaml_file)
        except InvalidSolutionConfigurationException as e:
            raise e

    def find(self, name):
        """
          Scan the service data to see if a service has been configured with the specified name.

          :param name: name of the service to scan for
          :type name: str

          :returns: the service data for a match or None if not found
          :rtype: ServiceData or None
        """
        for service_data in self.available_services:
            if service_data.name is not None and service_data.name == name:
                return service_data
        return None

    def unnamed(self):
        """
          Gather all service data which hasn't been provided overrides for the service name.
          These will be using their service default names. This is used by the service manager
          to get the list of services it needs to look up to find a corresponding match when
          enabling a service by name.

          :returns: service data items when do not have a name
          :rtype: [ServiceData]
        """
        return [s for s in self.available_services if s.name is None]
