#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import genpy
import rospkg
import rospy
from rocon_utilities.exceptions import ResourceNotFoundException
import yaml
import rocon_utilities
import concert_msgs.msg as concert_msgs
import unique_id

from .exceptions import InvalidServiceDescription

##############################################################################
# ServiceList
##############################################################################


def _service_filenames_from_list(service_list_resource, rospack):
    '''
      Service lists are stored in a .services file which contains
      a list of resource names pointing to individual service descriptions.

      Collect all the such here in one list of service description filenames.

      @param service_list_resource : resource location of a services file.
      @type string with ext .services)

      @param rospack - reuse the resource finding mechanism to speed up operations (has its own cache)
      @type rospkg.Rospack

      @raise rocon_utilities.exceptions.ResourceNotFoundException : if resource not found
    '''
    service_filenames = []
    try:
        services_filename = rocon_utilities.find_resource_from_string(service_list_resource, extension='services', rospack=rospack)
    except ResourceNotFoundException:
        # better error message
        raise ResourceNotFoundException("services resource not found [%s]" % service_list_resource)
    with open(services_filename) as f:
        services_yaml = yaml.load(f)
        for service_resource in services_yaml:
            try:
                service_filenames.append(rocon_utilities.find_resource_from_string(service_resource, extension='service', rospack=rospack))
            except ResourceNotFoundException as e:
                # Don't want to raise, just skip this filename and give a warning.
                rospy.logwarn("Service Manager : [%s]" % str(e))
    return service_filenames

#    service_filenames = []
#            # This can raise rocon_utilities.exceptions.ResourceNotFoundException
#            service_filenames.append(rocon_utilities.find_resource_from_string(service_resource, extension='service', rospack=rospack))


def load_service_descriptions_from_service_lists(service_lists):
    """
    Load service descriptions from a list of services lists. Take care that
    we are not adding two service descriptions with the same unique name.

    @param service_list_resource : resource name for the services file.
    @type str : e.g. concert_tutorial/tutorial.services

    @return the loaded service descriptions.
    @rtype { service_name : concert_msgs.ConcertService }
    """
    service_descriptions = {}
    rospack = rospkg.RosPack()
    service_filenames = []
    for service_list in service_lists:
        try:
            service_filenames.extend(_service_filenames_from_list(service_list, rospack))
        except ResourceNotFoundException as e:
            rospy.logwarn("Service Manager : [%s]" % str(e))
    for filename in service_filenames:
        with open(filename) as f:
            service_description = concert_msgs.ConcertService()
            service_yaml = yaml.load(f)
            genpy.message.fill_message_args(service_description, service_yaml)
            # Validation
            if service_description.launcher_type != concert_msgs.ConcertService.TYPE_ROSLAUNCH and \
               service_description.launcher_type != concert_msgs.ConcertService.TYPE_CUSTOM:
                rospy.logwarn("Service Manager : invalid service description [%s]" % (filename))
                continue
            # Fill in missing fields or modify correctly some values
            service_description.uuid = unique_id.toMsg(unique_id.fromRandom())
            # We need to make sure the service description name is a valid rosgraph namespace name
            # @todo Actually call the rosgraph function to validate this (do they have converters?)
            service_description.name = service_description.name.lower().replace(" ", "_")
            if service_description.name in service_descriptions.keys():
                rospy.logwarn("Service Manager : service description with this name already present, not adding [%s]" % service_description.name)
            else:
                service_descriptions[service_description.name] = service_description
    return service_descriptions.values()
