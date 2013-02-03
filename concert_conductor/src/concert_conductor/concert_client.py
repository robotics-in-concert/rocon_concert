#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/concert_conductor/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_app_manager_msgs.msg as rapp_manager_msgs
import rocon_app_manager_msgs.srv as rapp_manager_srvs
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs

##############################################################################
# Exceptions
##############################################################################


class ConcertClientException(Exception):
    """Exception class for concert client related errors"""
    pass

##############################################################################
# ClientInfo Class
##############################################################################


class ConcertClient(object):

    def __init__(self, client_name, param):
        '''
          Initialise, finally ending with a call on the service to the client's
          platform info service.

          @raise ConcertClientException : when platform info service is unavailable
        '''

        self.data = concert_msgs.ConcertClient()
        self._rawdata = {}
        self.name = client_name
        self.param = param

        self.platform_info = None
        self.service_execution = {}  # Services to execute, e.g. start_app, stop_app

        self._get_client_info()

        #### Setup Invitation                        name, type
        self.invitation = rospy.ServiceProxy(str(self.name + '/' + param['invitation'][0]), param['invitation'][1])

        # Don't wait for these - not up till invited
        for k in param['execution']['srv'].keys():
            key = param['execution']['srv'][k][0]
            service_type = param['execution']['srv'][k][1]
            self.service_execution[k] = rospy.ServiceProxy(str(self.name + '/' + key), service_type)

        try:
            self._update()
        except ConcertClientException:
            raise

    def _get_client_info(self):
        '''
          Pulls platform information from the advertised platform_info on another
          ros system. It is just a one-shot only used at construction time.

          It pulls platform_info and list_apps information.

          It also sets up a service for checking the status of the client (self._status_service)
        '''
        rospy.loginfo("Conductor: retrieving client information [%s]" % self.name)
        pull_service = rospy.ServiceProxy('~pull', gateway_srvs.Remote)
        req = gateway_srvs.RemoteRequest()
        req.cancel = False
        req.remotes = []
        for service_name in ['platform_info', 'list_apps', 'status']:
            rule = gateway_msgs.Rule()
            rule.name = str(self.name + '/' + service_name)
            rule.node = ''
            rule.type = gateway_msgs.ConnectionType.SERVICE
            req.remotes.append(gateway_msgs.RemoteRule(self.name.lstrip('/'), rule))
        resp = pull_service(req)
        if resp.result != 0:
            rospy.logwarn("Conductor: failed to pull the platform info service from the client.")
            return None
        # Platform_info
        platform_info_service = rospy.ServiceProxy(str(self.name + '/' + 'platform_info'), rapp_manager_srvs.GetPlatformInfo)
        list_app_service = rospy.ServiceProxy(str(self.name + '/' + 'list_apps'), rapp_manager_srvs.GetAppList)
        # This one is permanent
        self._status_service = rospy.ServiceProxy(str(self.name + '/' + 'status'), rapp_manager_srvs.Status)
        try:
            platform_info_service.wait_for_service()
            list_app_service.wait_for_service()
            self._status_service.wait_for_service()
        except rospy.ServiceException, e:
            raise e
        platform_info = platform_info_service().platform_info
        self.data.name = platform_info.name
        self.data.platform = platform_info.platform
        self.data.system = platform_info.system
        self.data.robot = platform_info.robot
        # List Apps
        try:
            list_app_service.wait_for_service()
        except rospy.ServiceException, e:
            raise e
        self.data.apps = list_app_service().apps

    def to_msg_format(self):
        '''
          Returns the updated client information status in ros-consumable msg format.

          @return the client information as a ros message.
          @rtype concert_msgs.ConcertClient

          @raise rospy.service.ServiceException : when assumed service link is unavailable
        '''
        try:
            self._update()
        except ConcertClientException:
            raise
        return self.data

    def _update(self):
        '''
          Adds the current client status to the platform_info, list_apps information already
          retrieved from the client and puts them in a convenient msg format,
          ready for exposing outside the conductor (where? I can't remember).

          @raise rospy.service.ServiceException : when assumed service link is unavailable
        '''
        try:
            self._rawdata['status'] = self._status_service(rapp_manager_srvs.StatusRequest())
        except rospy.service.ServiceException:
            raise ConcertClientException("client platform information services unavailable (disconnected?)")

        self.data.name = self._rawdata['status'].namespace
        self.data.last_connection_timestamp = rospy.Time.now()
        if self._rawdata['status'].remote_controller == rapp_manager_msgs.Constants.NO_REMOTE_CONNECTION:
            self.data.client_status = concert_msgs.Constants.CONCERT_CLIENT_STATUS_AVAILABLE
        # Todo - fix this
        #elif self._rawdata['status'].remote_controller == _this_concert_name:
        #    self.data.client_status = concert_msgs.Constants.CONCERT_CLIENT_STATUS_CONNECTED
        else:
            self.data.client_status = concert_msgs.Constants.CONCERT_CLIENT_STATUS_CONNECTED
        #    self.data.client_status = concert_msgs.Constants.CONCERT_CLIENT_STATUS_UNAVAILABLE

    def invite(self, name, ok_flag):
        req = concert_srvs.InvitationRequest(name, ok_flag)
        resp = self.invitation(req)

        if resp.success == True:
            self.set_channel()
        else:
            raise Exception(str("Invitation Failed : " + self.name))

    def start_app(self, app_name, remappings):
        '''
          @param app_name : string unique identifier for the app
          @type string
          @param remappings : list of (from,to) pairs
          @type list of tuples
        '''
        self.service_execution['start_app'].wait_for_service()
        req = rapp_manager_srvs.StartAppRequest()
        req.name = app_name
        req.remappings = []
        for remapping in remappings:
            req.remappings.append(rapp_manager_msgs.Remapping(remapping[0], remapping[1]))
        unused_resp = self.service_execution['start_app'](req)

    def set_channel(self):
        param = self.param
        # Services
        self.service_exec = {}
        for k in param['execution']['srv'].keys():
            key = param['execution']['srv'][k][0]
            type = param['execution']['srv'][k][1]
            self.service_exec[k] = rospy.ServiceProxy(str(self.name + '/' + key), type)
