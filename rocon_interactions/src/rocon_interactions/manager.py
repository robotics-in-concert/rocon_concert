#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import uuid
import rospy
import rosgraph
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs
import unique_id
import rocon_utilities

# Local imports
import remocon_app_utils
from .remocon_monitor import RemoconMonitor

##############################################################################
# Role Manager
##############################################################################


class RoleManager(object):
    '''
      Manages connectivity information provided by services and provides this
      for human interactive agent (aka remocon) connections.
    '''
    __slots__ = [
            'role_and_app_table',  # Dictionary of string : concert_msgs.RemoconApp[]
            'publishers',
            'parameters',
            'services',
            'spin',
            'platform_info',
            '_watch_loop_period',
            '_remocon_monitors'  # list of currently connected remocons.
        ]

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self):
        self.role_and_app_table = {}
        self.publishers = self._setup_publishers()
        self.services = self._setup_services()
        self.parameters = self._setup_parameters()
        self._watch_loop_period = 1.0
        self._remocon_monitors = {}  # topic_name : RemoconMonitor

    def spin(self):
        '''
          Parse the set of /remocons/<name>_<uuid> connections.
        '''
        while not rospy.is_shutdown():
            master = rosgraph.Master(rospy.get_name())
            diff = lambda l1, l2: [x for x in l1 if x not in l2]
            try:
                # This master call returns a filtered list of [topic_name, topic_type] elemnts (list of lists)
                remocon_topics = [x[0] for x in master.getPublishedTopics(concert_msgs.Strings.REMOCONS_NAMESPACE)]
                new_remocon_topics = diff(remocon_topics, self._remocon_monitors.keys())
                lost_remocon_topics = diff(self._remocon_monitors.keys(), remocon_topics)
                for remocon_topic in new_remocon_topics:
                    self._remocon_monitors[remocon_topic] = RemoconMonitor(remocon_topic, self._ros_publish_interactive_clients)
                    self._ros_publish_interactive_clients()
                    rospy.loginfo("Role Manager : new remocon connected [%s]" % remocon_topic[len(concert_msgs.Strings.REMOCONS_NAMESPACE) + 1:])  # strips the /remocons/ part
                for remocon_topic in lost_remocon_topics:
                    self._remocon_monitors[remocon_topic].unregister()
                    del self._remocon_monitors[remocon_topic]  # careful, this mutates the dictionary http://stackoverflow.com/questions/5844672/delete-an-element-from-a-dictionary
                    self._ros_publish_interactive_clients()
                    rospy.loginfo("Role Manager : remocon left [%s]" % remocon_topic[len(concert_msgs.Strings.REMOCONS_NAMESPACE) + 1:])  # strips the /remocons/ part
            except rosgraph.masterapi.Error:
                rospy.logerr("Role Manager : error trying to retrieve information from the local master.")
            except rosgraph.masterapi.Failure:
                rospy.logerr("Role Manager : failure trying to retrieve information from the local master.")
            rospy.rostime.wallsleep(self._watch_loop_period)

    def _setup_publishers(self):
        '''
          These are all public topics. Typically that will drop them into the /concert
          namespace.
        '''
        publishers = {}
        publishers['roles'] = rospy.Publisher('~roles', concert_msgs.Roles, latch=True)
        publishers['interactive_clients'] = rospy.Publisher('~interactive_clients', concert_msgs.InteractiveClients, latch=True)
        return publishers

    def _setup_services(self):
        '''
          These are all public services. Typically that will drop them into the /concert
          namespace.
        '''
        services = {}
        services['get_roles_and_apps'] = rospy.Service('~get_roles_and_apps',
                                                       concert_srvs.GetRolesAndApps,
                                                       self._ros_service_filter_roles_and_apps)
        services['get_app'] = rospy.Service('~get_app',
                                                       concert_srvs.GetApp,
                                                       self._ros_service_get_app)
        services['set_roles_and_apps'] = rospy.Service('~set_roles_and_apps',
                                                       concert_srvs.SetRolesAndApps,
                                                       self._ros_service_set_roles_and_apps)
        services['request_interaction'] = rospy.Service('~request_interaction',
                                                       concert_srvs.RequestInteraction,
                                                       self._ros_service_request_interaction)
        return services

    def _setup_parameters(self):
        param = {}
        #param['auto_invite'] = rospy.get_param('~auto_invite', False)
        return param

    ##########################################################################
    # Ros Api Functions
    ##########################################################################

    def _ros_service_filter_roles_and_apps(self, request):
        '''
          Handle incoming requests to provide a role-applist dictionary
          filtered for the requesting platform.
        '''
        roles_and_apps = concert_srvs.GetRolesAndAppsResponse()
        roles_and_apps.data = []
        if request.roles:  # works for None or empty list
            # Validate that each role is in self.role_and_app_table here
            roles = request.roles
        else:
            roles = self.role_and_app_table.keys()
        for role in roles:
            role_app_list = concert_msgs.RoleAppList(role, [])
            for app in self.role_and_app_table[role]:
                if remocon_app_utils.is_runnable(app, request.platform_info):
                    role_app_list.remocon_apps.append(app)
            roles_and_apps.data.append(role_app_list)
        #print 'roles_and_apps service result: %s' % roles_and_apps
        return roles_and_apps

    def _ros_publish_interactive_clients(self):
        interactive_clients = concert_msgs.InteractiveClients()
        for remocon in self._remocon_monitors.values():
            if remocon.status is not None:  # i.e. we are monitoring it.
                interactive_client = concert_msgs.InteractiveClient()
                interactive_client.name = remocon.name
                interactive_client.id = unique_id.toMsg(uuid.UUID(remocon.status.uuid))
                interactive_client.platform_info = rocon_utilities.platform_info.to_string(remocon.status.platform_info)
                if remocon.status.running_app:
                    interactive_client.app_name = remocon.status.app_name
                    interactive_clients.running_clients.append(interactive_client)
                else:
                    interactive_clients.idle_clients.append(interactive_client)
        self.publishers['interactive_clients'].publish(interactive_clients)

    def _ros_service_get_app(self, request):
        '''
          Handle incoming requests for a single app.
        '''
        response = concert_srvs.GetAppResponse()
        response.result = False
        for role in self.role_and_app_table.keys():
            for app in self.role_and_app_table[role]:
                if request.hash == app.hash:
                    if remocon_app_utils.is_runnable(app, request.platform_info):
                        response.app = app
                        response.result = True
                        break
        return response

    def _ros_service_set_roles_and_apps(self, request):
        '''
          Add or remove role-app entries from the role-app table.

          Note: only setting for the moment.
          Note: uniquely identifying apps by name (not very sane).

          @param request list of roles-apps to set
          @type concert_srvs.SetRolesAndAppsRequest
        '''
        role_app_lists = request.data  # concert_msgs.RoleAppList[]
        if request.add:
            for role_app_list in role_app_lists:  # concert_msgs.RemoconApp[]
                role = role_app_list.role
                if not role in self.role_and_app_table.keys():
                    rospy.loginfo("Role Manager : creating a new role [%s]" % role)
                    self.role_and_app_table[role] = []
                    self.publishers['roles'].publish(concert_msgs.Roles(self.role_and_app_table.keys()))
                for app in role_app_list.remocon_apps:
                    if remocon_app_utils.is_app_in_app_list(app, self.role_and_app_table[role]) is None:
                        self.role_and_app_table[role].append(app)
                        rospy.loginfo("Role Manager : adding to the app list [%s][%s]" % (role, app.name))
        else: # clean
            for role_app_list in role_app_lists:  # concert_msgs.RemoconApp[]
                role = role_app_list.role
                for app in role_app_list.remocon_apps:
                    if role in self.role_and_app_table.keys():
                        app = remocon_app_utils.is_app_in_app_list(app, self.role_and_app_table[role])
                        if app is not None:
                            self.role_and_app_table[role].remove(app)
        response = concert_srvs.SetRolesAndAppsResponse()
        response.result = True
        return response

    def _ros_service_request_interaction(self, request):
        response = concert_srvs.RequestInteractionResponse()
        response.result = True
        response.error_code = concert_msgs.ErrorCodes.SUCCESS
        maximum_quota = None
        if request.role in self.role_and_app_table.keys():
            for app in self.role_and_app_table[request.role]:  # app is concert_msgs.RemoconApp
                if app.name == request.application and app.service_name == request.service_name:
                    if app.max == 0:
                        return response
                    else:
                        maximum_quota = app.max
                        break
        if maximum_quota is not None:
            count = 0
            for remocon_monitor in self._remocon_monitors.values():
                if remocon_monitor.status is not None and remocon_monitor.status.running_app:
                    # Todo this is a weak check as it is not necessarily uniquely identifying the app
                    if remocon_monitor.status.app_name == request.application:
                        count += 1
            if count < max:
                return response
            else:
                response.error_code = concert_msgs.ErrorCodes.ROLE_APP_QUOTA_REACHED
                response.message = concert_msgs.ErrorCodes.MSG_ROLE_APP_QUOTA_REACHED
        else:
            response.error_code = concert_msgs.ErrorCodes.ROLE_APP_UNAVAILABLE
            response.message = concert_msgs.ErrorCodes.MSG_ROLE_APP_UNAVAILABLE
        response.result = False
        return response
