#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_orchestration/concert_conductor/LICENSE
#
##############################################################################
# Imports
##############################################################################

import sys
import traceback
import roslib
roslib.load_manifest('concert_conductor')
import rospy
import rosservice
import appmanager_msgs.srv as appmanager_srvs
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs
import gateway_msgs.srv as gateway_srvs
from .concert_client import ClientInfo, ConcertClientException

##############################################################################
# Conductor
##############################################################################


class Conductor(object):

    def __init__(self):
        ##################################
        # Pubs, Subs and Services
        ##################################
        self.publishers = {}
        self.publishers["concert_clients"] = rospy.Publisher("~concert_clients", concert_msgs.ConcertClients, latch=True)
        self.srv = {}
        #self.srv['concert_clients'] = rospy.Service('~concert_clients', concert_srvs.ClientList, self.processClientList)
        self.srv['invite'] = rospy.Service('~invite', concert_srvs.Invite, self.processInvite)

        ##################################
        # Variables
        ##################################
        self._concert_clients = {}  # List of concert_clients, both visible and out of range
        self._invited_clients = {}  # Clients that have previously been invited, but disappeared
        self._bad_clients = []  # Used to remember clients that are bad.so we don't try and pull them again
        self._concert_name = None
        self._watcher_period = 1.0  # Period for the watcher thread (i.e. update rate)

        ##################################
        # Initialisation
        ##################################
        self._get_concert_name()
        self._setup_ros_parameters()

    def processClientList(self, req):
        out = [self._concert_clients[cinfo].get_client() for cinfo in self._concert_clients]
        return concert_srvs.ClientListResponse(out)

    def processInvite(self, req):
        mastername = req.mastername
        resp = self.invite(mastername, req.clientnames, req.ok_flag)
        return concert_srvs.InviteResponse("Success to invite[" + str(resp) + "] : " + str(req.clientnames))

    def invite(self, mastername, clientnames, ok_flag):
        try:
            for name in clientnames:
                if not name.startswith('/'):
                    name = '/' + name
                unused_resp = self._concert_clients[name].invite(mastername, ok_flag)
                rospy.loginfo("Conductor : successfully invited [%s]" % str(name))
                self._invited_clients[name] = ok_flag
        except Exception as e:
            rospy.logerr("Conductor : %s" % str(e))
            return False
        return True

    def spin(self):
        '''
          Maintains the clientele list.
        '''
        while not rospy.is_shutdown():
            visible_clients = self._get_visible_clients()
            self._prune_client_list(visible_clients)
            new_clients = [c for c in visible_clients if (c not in self._concert_clients) and (c not in self._bad_clients)]

            # Create new clients info instance
            for new_client in new_clients:
                try:
                    rospy.loginfo("Conductor : new client found [%s]" % new_client)
                    self._concert_clients[new_client] = ClientInfo(new_client, self.param)

                    # re-invitation of clients that disappeared and came back
                    if new_client in self._invited_clients:
                        self.invite(self._concert_name, [new_client], True)
                except Exception as e:
                    self._bad_clients.append(new_client)
                    rospy.loginfo("Conductor : failed to establish client [%s][%s]" % (str(new_client), str(e)))

            if self.param['config']['auto_invite']:
                client_list = [client for client in self._concert_clients
                                     if (client not in self._invited_clients)
                                     or (client in self._invited_clients and self._invited_clients[client] == False)]
                self.invite(self._concert_name, client_list, True)
            self._publish_discovered_concert_clients()
            rospy.sleep(self._watcher_period)

    ###########################################################################
    # Helpers
    ###########################################################################

    def _get_visible_clients(self):
        '''
          Return the list of clients currently visible on network. This
          currently just checks the local system for any topic
          ending with 'invitation'...which should be representative of a
          concert client.
        '''
        suffix = self.param['invitation'][0]
        services = rosservice.get_service_list()
        visible_clients = [s[:-(len(suffix) + 1)] for s in services if s.endswith(suffix)]
        return visible_clients

    def _prune_client_list(self, new_clients):
        '''
          Remove from the current client list any whose topics and services have disappeared.
        '''
        for name in self._concert_clients.keys():
            if not name in new_clients:
                rospy.loginfo("Conductor : client left : " + name)
                del self._concert_clients[name]

    def _publish_discovered_concert_clients(self):
        '''
            Provide a list of currently discovered clients. This goes onto a
            latched publisher, so subscribers will always have the latest
            without the need to poll a service.
        '''
        discovered_concert = concert_msgs.ConcertClients()
        for unused_client, client_info in self._concert_clients.iteritems():
            try:
                discovered_concert.clients.append(client_info.to_msg_format())
            except ConcertClientException:
                # service was broken, quietly do not add it
                # (it will be deleted from client list next pass)
                pass
        self.publishers["concert_clients"].publish(discovered_concert)

    ###########################################################################
    # Private Initialisation
    ###########################################################################

    def _get_concert_name(self):
        # Get concert name (i.e. gateway name)
        gateway_info_service = rospy.ServiceProxy("~gateway_info", gateway_srvs.GatewayInfo)
        gateway_info_service.wait_for_service()
        gateway_is_connected = False

        while not rospy.is_shutdown() and not gateway_is_connected:
            gateway_info = gateway_info_service(gateway_srvs.GatewayInfoRequest())
            gateway_is_connected = gateway_info.connected
            if gateway_info.connected == True:
                self._concert_name = gateway_info.name
            else:
                rospy.loginfo("Conductor : no hub yet available, spinning...")
            rospy.sleep(1.0)

    def _setup_ros_parameters(self):
        param = {}
        param['config'] = {}
        param['config']['auto_invite'] = rospy.get_param('~auto_invite', False)

        param['invitation'] = (rospy.get_param('~invitation', 'invitation'), concert_srvs.Invitation)
        param['info'] = {}
        param['info']['list_apps'] = (rospy.get_param('~list_apps', 'list_apps'), appmanager_srvs.GetAppList)
        param['info']['platform_info'] = (rospy.get_param('~platform_info', 'platform_info'), concert_srvs.GetPlatformInfo)
        param['info']['status'] = (rospy.get_param('~status', 'status'), concert_srvs.Status)

        param['execution'] = {}
        param['execution']['srv'] = {}
        param['execution']['srv']['start_app'] = (rospy.get_param('~start_app', 'start_app'), appmanager_srvs.StartApp)
        param['execution']['srv']['stop_app'] = (rospy.get_param('~stop_app', 'stop_app'), appmanager_srvs.StopApp)

        self.param = param
