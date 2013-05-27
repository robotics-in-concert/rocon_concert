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

    def __init__(self, client_name, gateway_name, is_local_client=False):
        '''
          Initialise, finally ending with a call on the service to the client's
          platform info service.

          @param client_name : a string (+ index) human consumable form of the name
          @type str
          @param gateway_name : the name with 16 byte hash attached
          @type str
          @param is_local_client : differentiates an interactive local client from a gateway client
          @type bool

          @raise ConcertClientException : when platform info service is unavailable
        '''

        self.data = concert_msgs.ConcertClient()
        self.gateway_name = gateway_name  # this is the rather unsightly name + hash key
        self.name = client_name  # usually name (+ index), a more human consumable name

        self.platform_info = None
        self.service_execution = {}  # Services to execute, e.g. start_app, stop_app
        self._is_local_client = is_local_client  # As opposed to a gateway client
        if not self._is_local_client:
            self._pull_concert_client()  # get concert client info and pull required handles in
            # could maybe catch an exception on error here
        self._init()
        self._setup_service_proxies()

        try:
            self._update()
        except ConcertClientException:
            raise

    def _pull_concert_client(self):
        '''
          Pulls platform information from the advertised platform_info on another
          ros system. It is just a one-shot only used at construction time.

          It pulls platform_info and list_apps information.

          It also pulls the required handles in for further manipulation ('status' and 'invite')
        '''
        rospy.loginfo("Conductor: retrieving client information [%s]" % self.name)
        pull_service = rospy.ServiceProxy('~pull', gateway_srvs.Remote)
        req = gateway_srvs.RemoteRequest()
        req.cancel = False
        req.remotes = []
        for service_name in ['platform_info', 'list_apps', 'status', 'invite']:
            rule = gateway_msgs.Rule()
            rule.name = str('/' + self.gateway_name + '/' + service_name)
            rule.node = ''
            rule.type = gateway_msgs.ConnectionType.SERVICE
            req.remotes.append(gateway_msgs.RemoteRule(self.gateway_name.lstrip('/'), rule))
        resp = pull_service(req)
        if resp.result != 0:
            rospy.logwarn("Conductor: failed to pull the platform info service from the client.")
            return None

    def _init(self):
        # Platform_info
        platform_info_service = rospy.ServiceProxy(str('/' + self.gateway_name + '/' + 'platform_info'), rapp_manager_srvs.GetPlatformInfo)
        list_app_service = rospy.ServiceProxy(str('/' + self.gateway_name + '/' + 'list_apps'), rapp_manager_srvs.GetAppList)
        # These are permanent
        self._status_service = rospy.ServiceProxy('/' + str(self.gateway_name + '/' + 'status'), rapp_manager_srvs.Status)
        self._invite_service = rospy.ServiceProxy('/' + str(self.gateway_name + '/' + 'invite'), rapp_manager_srvs.Invite)
        try:
            platform_info_service.wait_for_service()
            list_app_service.wait_for_service()
            self._status_service.wait_for_service()
            self._invite_service.wait_for_service()
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
            status = self._status_service(rapp_manager_srvs.StatusRequest())
        except rospy.service.ServiceException:
            raise ConcertClientException("client platform information services unavailable (disconnected?)")
        self.data.name = self.name  # use the human friendly name
        self.data.gateway_name = self.gateway_name
        #self.data.name = status.namespace
        self.data.last_connection_timestamp = rospy.Time.now()
        if status.remote_controller == rapp_manager_msgs.Constants.NO_REMOTE_CONNECTION:
            self.data.client_status = concert_msgs.Constants.CONCERT_CLIENT_STATUS_AVAILABLE
        # Todo - fix this
        #elif status.remote_controller == _this_concert_name:
        #    self.data.client_status = concert_msgs.Constants.CONCERT_CLIENT_STATUS_CONNECTED
        else:
            self.data.client_status = concert_msgs.Constants.CONCERT_CLIENT_STATUS_CONNECTED
        #    self.data.client_status = concert_msgs.Constants.CONCERT_CLIENT_STATUS_UNAVAILABLE

    def invite(self, concert_gateway_name, client_local_name, ok_flag):
        '''
          Bit messy with ok_flag here as we are mid-transition to using 'cancel' flag in the
          invite services.

          @param concert_gateway_name : have to let the client know the concert gateway name
                                        so they can flip us topics...
          @type str
        '''
        req = rapp_manager_srvs.InviteRequest(concert_gateway_name, client_local_name, not ok_flag)
        resp = self._invite_service(req)

        if resp.result == True:
            self._setup_service_proxies()
        else:
            raise Exception(str("Invitation Failed : " + self.name))

    def start_app(self, app_name, remappings):
        '''
          @param app_name : string unique identifier for the app
          @type string
          @param remappings : list of (from,to) pairs
          @type list of tuples
        '''
        self._start_app_service.wait_for_service()
        req = rapp_manager_srvs.StartAppRequest()
        req.name = app_name
        req.remappings = []
        for remapping in remappings:
            req.remappings.append(rapp_manager_msgs.Remapping(remapping[0], remapping[1]))
        unused_resp = self._start_app_service(req)

    def _setup_service_proxies(self):
        self._start_app_service = rospy.ServiceProxy(str(self.name + '/start_app'), rapp_manager_srvs.StartApp)
        self._stop_app_service = rospy.ServiceProxy(str(self.name + '/stop_app'), rapp_manager_srvs.StopApp)
