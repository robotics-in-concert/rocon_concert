# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import genpy
import rospy
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs
import rocon_utilities
import yaml

from .names import topics
from .exceptions import InvalidRoleAppYaml

##############################################################################
# Utilities
##############################################################################


def load_role_apps_from_yaml(role_app_yaml_resource, service_name):
    role_app_lists = []
    try:
        yaml_filename = rocon_utilities.find_resource_from_string(role_app_yaml_resource)
    except IOError as e:  # resource not found.
        raise rocon_utilities.exceptions.ResourceNotFoundException(str(e))
    with open(yaml_filename) as f:
        role_app_yaml = yaml.load(f)
    for role_app_list_yaml in role_app_yaml:
        role_app_list = concert_msgs.RoleAppList()
        genpy.message.fill_message_args(role_app_list, role_app_list_yaml)
        try:
            role_app_list = _finalise_role_app_list(role_app_list, service_name)
        except InvalidRoleAppYaml as e:
            raise e
        role_app_lists.append(role_app_list)
    return role_app_lists


def _finalise_role_app_list(role_app_list, service_name):
    '''
      Ideally we would like to do the following things here:

        - validate the role_app_list
        - check if non-compulsory fields have been set, if not set defaults
        - extract the icon from a resource name
    '''
    # Validate
    for remocon_app in role_app_list.remocon_apps:
        if remocon_app.max < -1:
            raise InvalidRoleAppYaml("maximum instance configuration cannot be negative [%s]" % remocon_app.display_name)
        # Set sane defaults
        if remocon_app.max == 0:
            remocon_app.max = 1
        if remocon_app.icon.resource_name == "":
            remocon_app.icon.resource_name = 'concert_master/rocon_text.png'
        remocon_app.icon = rocon_utilities.icon_resource_to_msg(remocon_app.icon.resource_name)
        remocon_app.service_name = service_name
    return role_app_list


##############################################################################
# RoleAppLoader
##############################################################################

class RoleAppLoader(object):
    '''
      This class is responsible for loading the role manager with the roles
      and app specifications provided in the service definitions.

      @todo in the future this will also allow the solution to override
      various specifications - roles, app display names, descriptions.
    '''
    __slots__ = [
            '_set_roles_and_apps_proxy',
        ]

    def __init__(self):
        '''
        Don't do any loading here, just set up infrastructure and overrides from
        the solution.
        '''
        self._set_roles_and_apps_proxy = rospy.ServiceProxy(topics.set_roles_and_apps, concert_srvs.SetRolesAndApps)
        try:
            self._set_roles_and_apps_proxy.wait_for_service(5.0)
        except rospy.exceptions.ROSException:
            # timeout exceeded
            raise rospy.exceptions.ROSException("timed out waiting for service [%s]" % self._set_roles_and_apps_proxy.resolved_name)
        except rospy.exceptions.ROSInterruptException as e:
            raise e
        # In the future we will load this up with the solution overrides
        # for the roles.

    def load(self, role_app_yaml_resource, service_name, load=True):
        '''
        Parse the service description's configuration of roles/apps and
        pass these along to the role manager.

        @param role_app_yaml_resource : yaml resource name for role-app parameterisation
        @type yaml string

        @param service_name: (unique) name of the service loading the information (used for namespacing by the apps)
        @type string (e.g. turtles)

        @param load : either set, or remove the role-app information.
        @type boolean
        '''
        request = concert_srvs.SetRolesAndAppsRequest()
        request.add = load
        try:
            request.data = load_role_apps_from_yaml(role_app_yaml_resource, service_name)
        except rocon_utilities.exceptions.ResourceNotFoundException as e:
            raise e
        except InvalidRoleAppYaml as e:
            raise e
        unused_response = self._set_roles_and_apps_proxy(request)
