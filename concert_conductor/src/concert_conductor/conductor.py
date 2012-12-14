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
import concert_msgs.srv as concert_srvs
from .client_info import ClientInfo

##############################################################################
# Conductor
##############################################################################


class Conductor(object):

    clients = {}
    invited_clients = {}
    mastername = None

    def __init__(self):
        self.srv = {}
        self.srv['clientlist'] = rospy.Service('~list_clients', concert_srvs.ClientList, self.processClientList)
        self.srv['invite'] = rospy.Service('~invite', concert_srvs.Invite, self.processInvite)
        self.srv['set_auto_invite'] = rospy.Service('~set_auto_invite', concert_srvs.SetAutoInvite, self.processAutoInvite)

        self.parse_params()
        self._bad_clients = []  # Used to remember clients that are bad.so we don't try and pull them again

    def parse_params(self):
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

    def _get_clients(self):
        '''
          Return the list of clients found on network. This
          currently just checks the local system for any topic
          ending with 'invitation'...which should be representative of a
          concert client.
        '''
        suffix = self.param['invitation'][0]
        services = rosservice.get_service_list()
        visible_clients = [s[:-(len(suffix) + 1)] for s in services if s.endswith(suffix)]
        return visible_clients

    def _maintain_clientlist(self, new_clients):
        for name in self.clients.keys():
            if not name in new_clients:
                rospy.loginfo("Conductor : client left : " + name)
                del self.clients[name]

    def processClientList(self, req):
        out = [self.clients[cinfo].get_client() for cinfo in self.clients]

        return concert_srvs.ClientListResponse(out)

    def processInvite(self, req):
        mastername = req.mastername

        resp = self.invite(mastername, req.clientnames, req.ok_flag)

        return concert_srvs.InviteResponse("Success to invite[" + str(resp) + "] : " + str(req.clientnames))

    def processAutoInvite(self, req):
        rospy.loginfo("Conductor : Auto Invitation : " + str(req.is_auto))
        self.mastername = req.mastername
        self.param['config']['auto_invite'] = req.is_auto
        return concert_srvs.SetAutoInviteResponse(True)

    def invite(self, mastername, clientnames, ok_flag):
        try:
            for name in clientnames:
                if not name.startswith('/'):
                    name = '/' + name
                unused_resp = self.clients[name].invite(mastername, ok_flag)
                rospy.loginfo("Conductor : Success to invite[" + str(ok_flag) + "] : " + str(name))
                self.invited_clients[name] = ok_flag
        except Exception as e:
            rospy.logerr("Conductor : %s" % str(e))
            return False

        return True

    def spin(self):
        '''
          Maintains the clientele list.
        '''
        while not rospy.is_shutdown():
            # Get all services and collect
            clients = self._get_clients()
#            self.log(str(clients))

            # remove clients that have left
            self._maintain_clientlist(clients)

            # filter existing client from new client list
            new_clients = [c for c in clients if (c not in self.clients) and (c not in self._bad_clients)]

            # Create new clients info instance
            for new_client in new_clients:
                try:
                    rospy.loginfo("Conductor : new client found : " + new_client)
                    self.clients[new_client] = ClientInfo(new_client, self.param)

                    if new_client in self.invited_clients:
                        self.invite(self.mastername, [new_client], True)
                except Exception as e:
                    self._bad_clients.append(new_client)
                    rospy.loginfo("Conductor : failed to establish client[" + str(new_client) + "] : " + str(e))

            if self.param['config']['auto_invite']:
                client_list = [client for client in self.clients if (client not in self.invited_clients) or (client in self.invited_clients and self.invited_clients[client] == False)]
                self.invite(self.mastername, client_list, True)

            rospy.sleep(1.0)
