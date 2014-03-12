# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os.path
import time
import yaml

import rospkg
import rospy
import rocon_python_utils
import rocon_console.console as console

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

      :raises: :exc:`concert_service_manager.InvalidSolutionConfigurationException` if the yaml provides invalid configuration
    """
    services = []
    # read
    with open(yaml_file) as f:
        service_list = yaml.load(f)
        for s in service_list:
            overrides = s['overrides'] if 'overrides' in s else None
            services.append(ServiceData(s['resource_name'], overrides))
    # validate
    identifiers = []
    for service in services:
        if service.name:
            identifier = service.name
        else:
            identifier = service.resource_name
        if identifier in identifiers:
            raise InvalidSolutionConfigurationException("service configuration found with duplicate names [%s]" % identifier)
        else:
            identifiers.append(identifier)
    return services

##############################################################################
# Classes
##############################################################################


class ServiceData(object):

#     Can't use slots with setattr flexibility
#     __slots__ = [
#             'resource_name',  # ros resource name for the service
#             # overridable service variables
#             'name',
#             'description',
#             'icon',
#             'priority',
#             'interactions'
#             'parameters',
#         ]

    override_keys = ['name', 'description', 'icon', 'priority', 'interactions', 'parameters']

    def __init__(self, resource_name, overrides):
        """
          Process data from the spec into variables.

          :param resource_name: pkg/name identifying a concert service
          :type resource_name: str

          :param overrides: dictionary of the possible overridable variables for a service, key set is bounded
          :type overrides: dic
        """
        self.resource_name = resource_name
        for key in ServiceData.override_keys:
            value = overrides[key] if overrides and key in overrides.keys() else None
            setattr(self, key, value)
        # could do some validation of each value

    def __str__(self):
        s = ''
        s += console.green + "%s" % self.resource_name + console.reset + '\n'
        for key in ServiceData.override_keys:
            if getattr(self, key) is not None:
                s += console.cyan + "     " + key + console.reset + ": " + console.yellow + "%s" % getattr(self, key) + console.reset + '\n'
        return s


class SolutionConfiguration(object):
    """
      Stores the current solution's service related configuration. This is
      obtained from a ros resource yaml file which typically specifies the
      list of services permitted to run as well as any overridable
      configuration they may have.
    """
    __slots__ = [
            '_yaml_file',      # os.path to solution configuration file
            '_last_modified',  # timestamp of last file modifications
            'services',        # [ServiceData]
        ]

    def __init__(self, resource_name):
        """
          Initialise the class with a pointer to the yaml that will be
          scanned and later monitored for changes that can be applied
          to a running concert.

          :param resource_name: pkg/filename of a yaml formatted service configuration for a solution
          :type resource_name: str

          :raises: :exc:`rospkg.ResourceNotFound` if resource_name cannot be resolved.
        """
        try:
            self._yaml_file = rocon_python_utils.ros.find_resource_from_string(resource_name)
        except rospkg.ResourceNotFound as e:
            rospy.logerr("Service Manager : could not find the solution's configuration for services [%s]" % resource_name)
            raise e
        self._last_modified = time.ctime(os.path.getmtime(self._yaml_file))
        self.services = load_solution_configuration(self._yaml_file)

    def __str__(self):
        s = ''
        for service in self.services:
            s += " - %s" % service
        return s

    def __len__(self):
        return len(self.services)

    def reload(self):
        modified_time = time.ctime(os.path.getmtime(self._yaml_file))
        if modified_time != self._last_modified:
            self.services = load_solution_configuration(self._yaml_file)
        else:
            pass  # already up to date
