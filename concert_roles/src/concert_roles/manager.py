#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

#import os
import rospy
import rosgraph
#import std_msgs.msg as std_msgs
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs
#import rocon_std_msgs.msg as rocon_std_msgs
#import rocon_std_msgs.srv as rocon_std_srvs
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
            'roles',
            'publishers',
            'parameters',
            'services',
            'spin',
            'platform_info',
            '_watch_loop_period',
            '_remocons_namespace',
            '_remocon_monitors'  # list of currently connected remocons.
        ]

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self):
        self.roles = {}
        self.publishers = self._setup_publishers()
        self.services = self._setup_services()
        self.parameters = self._setup_parameters()
        self._watch_loop_period = 1.0
        self._remocons_namespace = '/remocons'
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
                remocon_topics = [x[0] for x in master.getPublishedTopics(self._remocons_namespace)]
                new_remocon_topics = diff(remocon_topics, self._remocon_monitors.keys())
                lost_remocon_topics = diff(self._remocon_monitors.keys(), remocon_topics)
                for remocon_topic in new_remocon_topics:
                    self._remocon_monitors[remocon_topic] = RemoconMonitor(remocon_topic)
                    rospy.loginfo("Role Manager : new remocon connected [%s]" % remocon_topic[10:])  # strips the /remocons/ part
                for remocon_topic in lost_remocon_topics:
                    self._remocon_monitors[remocon_topic].unregister()
                    del self._remocon_monitors[remocon_topic]  # careful, this mutates the dictionary http://stackoverflow.com/questions/5844672/delete-an-element-from-a-dictionary
                    rospy.loginfo("Role Manager : remocon left [%s]" % remocon_topic[10:])  # strips the /remocons/ part
                # Clear the currently connected remocons
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
        publishers['roles'] = rospy.Publisher('roles', concert_msgs.Roles, latch=True)
        return publishers

    def _setup_services(self):
        '''
          These are all public services. Typically that will drop them into the /concert
          namespace.
        '''
        services = {}
        services['get_roles_and_apps'] = rospy.Service('get_roles_and_apps',
                                                       concert_srvs.GetRolesAndApps,
                                                       self._ros_service_filter_roles_and_apps)
        services['set_roles_and_apps'] = rospy.Service('set_roles_and_apps',
                                                       concert_srvs.SetRolesAndApps,
                                                       self._ros_service_set_roles_and_apps)
        services['request_interaction'] = rospy.Service('request_interaction',
                                                       concert_srvs.RequestInteraction,
                                                       self._ros_service_request_interaction)
        return services

    def _setup_parameters(self):
        param = {}
        #param['auto_invite'] = rospy.get_param('~auto_invite', False)
        return param

    ##########################################################################
    # Ros Service Callbacks
    ##########################################################################

    def _ros_service_filter_roles_and_apps(self, request):
        '''
          Handle incoming requests to provide a role-applist dictionary
          filtered for the requesting platform.
        '''
        roles_and_apps = concert_srvs.GetRolesAndAppsResponse()
        roles_and_apps.data = []
        if request.roles:  # works for None or empty list
            # Validate that each role is in self.roles here
            roles = request.roles
        else:
            roles = self.roles.keys()
        for role in roles:
            role_app_list = concert_msgs.RoleAppList(role, [])
            for app in self.roles[role]:
                if remocon_app_utils.is_runnable(app, request.platform_info):
                    role_app_list.remocon_apps.append(app)
            roles_and_apps.data.append(role_app_list)
        #print 'roles_and_apps service result: %s' % roles_and_apps
        return roles_and_apps

    def _ros_service_set_roles_and_apps(self, request):
        '''
          Add or remove role-app entries from the role-app table.

          Note: only setting for the moment.
          Note: uniquely identifying apps by name (not very sane).
        '''
        role_app_lists = request.data  # concert_msgs.RoleAppList[]
        for role_app_list in role_app_lists:
            role = role_app_list.role
            if not role in self.roles.keys():
                rospy.loginfo("Role Manager : creating a new role [%s]" % role)
                self.roles[role] = []
                self.publishers['roles'].publish(concert_msgs.Roles(self.roles.keys()))
            for app in role_app_list.remocon_apps:
                if not remocon_app_utils.is_app_in_app_list(app, self.roles[role]):
                    self.roles[role].append(app)
                    rospy.loginfo("Role Manager : adding app to the app list [%s][%s]" % (role, app.name))
        response = concert_srvs.SetRolesAndAppsResponse()
        response.result = True
        return response

    def _ros_service_request_interaction(self, request):
        response = concert_srvs.RequestInteractionResponse()
        response.result = True
        response.error_code = concert_msgs.ErrorCodes.SUCCESS
        return response
