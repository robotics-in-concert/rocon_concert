#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import genpy
import rospkg
import rospy
import rocon_utilities
from rocon_utilities.exceptions import ResourceNotFoundException
import yaml
import concert_msgs.msg as concert_msgs
import unique_id

from .exceptions import InvalidServiceDescription

##############################################################################
# ServiceList
##############################################################################


def load_service_profiles(service_resource_names):
    """
    Scan the package path looking for service exports and grab the ones
    we are interested in and load their service profiles.

    @param service_resource_names : list of services in resource name ('pkg/name') format (name has no extension)
    @type list of str

    @return the loaded service descriptions.
    @rtype { service_name : concert_msgs.ConcertService }
    """
    # scan the package path
    ros_package_path = os.getenv('ROS_PACKAGE_PATH', '')
    ros_package_path = [x for x in ros_package_path.split(':') if x]
    package_index = rocon_utilities.package_index_from_package_path(ros_package_path)
    service_filenames = []
    for package in package_index.values():
        for export in package.exports:
            if export.tagname == 'rocon_service':
                service_filename_relative_path = export.content
                resource_name = package.name + "/" + os.path.splitext(os.path.basename(service_filename_relative_path))[0]
                if resource_name in service_resource_names:
                    service_filename = os.path.join(os.path.dirname(package.filename), service_filename_relative_path)
                    if not os.path.isfile(service_filename):
                        rospy.logwarn("Service Manager : couldn't find service definition for exported service [%s]" % resource_name)
                        continue
                    service_filenames.append(service_filename)
    # load the service profiles
    service_profiles = {}
    for filename in service_filenames:
        with open(filename) as f:
            service_profile = concert_msgs.ConcertService()
            service_yaml = yaml.load(f)
            genpy.message.fill_message_args(service_profile, service_yaml)
            # Validation
            if service_profile.launcher_type != concert_msgs.ConcertService.TYPE_ROSLAUNCH and \
               service_profile.launcher_type != concert_msgs.ConcertService.TYPE_CUSTOM:
                rospy.logwarn("Service Manager : invalid service description [%s]" % (filename))
                continue
            # Fill in missing fields or modify correctly some values
            service_profile.uuid = unique_id.toMsg(unique_id.fromRandom())
            # We need to make sure the service description name is a valid rosgraph namespace name
            # @todo Actually call the rosgraph function to validate this (do they have converters?)
            service_profile.name = service_profile.name.lower().replace(" ", "_")
            if service_profile.name in service_profiles.keys():
                rospy.logwarn("Service Manager : service description with this name already present, not adding [%s]" % service_profile.name)
            else:
                service_profiles[service_profile.name] = service_profile
    return service_profiles.values()
