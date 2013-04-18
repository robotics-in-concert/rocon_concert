#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/concert_conductor/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_app_manager_msgs.srv as rapp_manager_srvs
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import rocon_utilities

# local imports
from .concert_client import ConcertClient, ConcertClientException

##############################################################################
# Conductor
##############################################################################


class Conductor(object):

    def __init__(self):
        ##################################
        # Pubs, Subs and Services
        ##################################
        self.publishers = {}
        # private spammy list_concert_clients publisher - used by web applications since they can't yet handle latched
        self.publishers["spammy_list_concert_clients"] = rospy.Publisher("~list_concert_clients", concert_msgs.ConcertClients)
        # efficient latched publisher, put in the public concert namespace.
        self.publishers["list_concert_clients"] = rospy.Publisher("list_concert_clients", concert_msgs.ConcertClients, latch=True)
        self.services = {}
        self.services['invite_concert_clients'] = rospy.Service('~invite_concert_clients', concert_srvs.Invite, self._process_invitation_request)
        # service clients
        self._remote_gateway_info_service = rospy.ServiceProxy("~remote_gateway_info", gateway_srvs.RemoteGatewayInfo)
        try:
            self._remote_gateway_info_service.wait_for_service()
        except rospy.ServiceException, e:
            raise e

        ##################################
        # Variables
        ##################################
        # Keys are client human friendly names, values the client class themselves
        self._concert_clients = {}  # List of concert_clients, both visible and out of range
        self._invited_clients = {}  # Clients that have previously been invited, but disappeared
        # List of gateway names identifying bad clients
        self._bad_clients = []  # Used to remember clients that are bad.so we don't try and pull them again
        self._concert_name = None
        self._watcher_period = 1.0  # Period for the watcher thread (i.e. update rate)

        ##################################
        # Initialisation
        ##################################
        self._get_concert_name()
        self._setup_ros_parameters()
        self._publish_discovered_concert_clients()  # Publish an empty list, to latch it and start

    def invite(self, concert_name, clientnames, ok_flag):
        for name in clientnames:
            try:
                unused_resp = self._concert_clients[name].invite(concert_name, name, ok_flag)
                rospy.loginfo("Conductor : successfully invited [%s]" % str(name))
                self._invited_clients[name] = ok_flag
            except KeyError:  # raised when name is not in the self._concert_clients keys
                rospy.logerr("Conductor : tried to invite unknown concert client [%s]" % name)
                return False
            except Exception as e:
                rospy.logerr("Conductor : failed to invite concert client [%s]" % str(e))
                return False
        return True

    def spin(self):
        '''
          Maintains the clientele list.
        '''
        while not rospy.is_shutdown():
            visible_clients = self._get_visible_clients()  # list of clients identified by gateway hash names
            number_of_pruned_clients = self._prune_client_list(visible_clients)
            number_of_new_clients = 0
            new_clients = [c for c in visible_clients if (c not in [client.gateway_name for client in self._concert_clients.values()])
                                                     and (c not in self._bad_clients)]

            # Create new clients info instance
            for gateway_name in new_clients:
                try:
                    # remove the 16 byte hex hash from the name
                    name = rocon_utilities.basename(gateway_name)
                    same_name_count = 0
                    for client in self._concert_clients.values():
                        if name == rocon_utilities.basename(client.gateway_name):
                            same_name_count += 1
                    if same_name_count != 0:
                        name = name + str(same_name_count + 1)
                    self._concert_clients[name] = ConcertClient(name, gateway_name, self.param)
                    rospy.loginfo("Conductor : new client found [%s]" % name)
                    number_of_new_clients += 1

                    # re-invitation of clients that disappeared and came back
                    if name in self._invited_clients:
                        self.invite(self._concert_name, [name], True)
                except Exception as e:
                    self._bad_clients.append(gateway_name)
                    rospy.loginfo("Conductor : failed to establish client [%s][%s]" % (str(name), str(e)))

            if self.param['config']['auto_invite']:
                client_list = [client for client in self._concert_clients
                                     if (client not in self._invited_clients)
                                     or (client in self._invited_clients and self._invited_clients[client] == False)]
                self.invite(self._concert_name, client_list, True)
            # Continually publish so it goes to web apps for now (inefficient).
            self._publish_discovered_concert_clients(self.publishers["spammy_list_concert_clients"])
            # Long term solution
            if number_of_pruned_clients != 0 or number_of_new_clients != 0:
                self._publish_discovered_concert_clients()
            rospy.sleep(self._watcher_period)

    ###########################################################################
    # Ros Callbacks
    ###########################################################################

    def _process_invitation_request(self, req):
        '''
          Handles service requests from the concert master to invite a set of concert clients.
        '''
        mastername = req.mastername
        resp = self.invite(mastername, req.clientnames, req.ok_flag)
        return concert_srvs.InviteResponse("Success to invite[" + str(resp) + "] : " + str(req.clientnames))

    ###########################################################################
    # Helpers
    ###########################################################################

    def _get_visible_clients(self):
        '''
          Return the list of clients currently visible on network. This
          currently just checks the remote gateways for 'platform_info'
          services ...which should be representative of a
          concert client (aye, not real safe like).

          @return visible_clients : list of visible clients identified by their gateway hash names
          @rtype list of str
        '''
        suffix = 'platform_info'
        visible_clients = []
        remote_gateway_info = self._remote_gateway_info_service()
        for remote_gateway in remote_gateway_info.gateways:
            for rule in remote_gateway.public_interface:
                if rule.name.endswith(suffix):
                    visible_clients.append(rule.name[1:-(len(suffix) + 1)])  # escape the initial '/' and the trailing '/platform_info'
        return visible_clients

    def _prune_client_list(self, new_clients):
        '''
          Remove from the current client list any whose topics and services have disappeared.

          @param new_clients : a list of new clients identified by their gateway hash names
          @type list of str
          @return number of pruned clients
          @rtype int
        '''
        number_of_pruned_clients = 0
        # Gateway names are unique hashed names, client names are also unique, but more human friendly
        for (client_name, gateway_name) in [(name, client.gateway_name) for name, client in self._concert_clients.items()]:
            if not gateway_name in new_clients:
                number_of_pruned_clients += 1
                rospy.loginfo("Conductor : client left : " + client_name)
                del self._concert_clients[client_name]
        return number_of_pruned_clients

    def _publish_discovered_concert_clients(self, list_concert_clients_publisher=None):
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
        if not list_concert_clients_publisher:
            list_concert_clients_publisher = self.publishers["list_concert_clients"]  # default
        list_concert_clients_publisher.publish(discovered_concert)

    ###########################################################################
    # Private Initialisation
    ###########################################################################

    def _get_concert_name(self):
        # Get concert name (i.e. gateway name)
        gateway_info_proxy = rocon_utilities.SubscriberProxy("~gateway_info", gateway_msgs.GatewayInfo)
        try:
            gateway_info_proxy.wait_for_publishers()
        except rospy.exceptions.ROSInterruptException:
            rospy.logwarn("Conductor : ros shut down before gateway info could be found.")

        while not rospy.is_shutdown():
            gateway_info = gateway_info_proxy(rospy.Duration(0.2))
            if gateway_info:
                if gateway_info.connected:
                    self._concert_name = gateway_info.name
                    break
                else:
                    rospy.loginfo("Conductor : no hub yet available, spinning...")
            rospy.sleep(1.0)

    def _setup_ros_parameters(self):
        param = {}
        param['config'] = {}
        param['config']['auto_invite'] = rospy.get_param('~auto_invite', False)

        param['execution'] = {}
        param['execution']['srv'] = {}
        param['execution']['srv']['start_app'] = (rospy.get_param('~start_app', 'start_app'), rapp_manager_srvs.StartApp)
        param['execution']['srv']['stop_app'] = (rospy.get_param('~stop_app', 'stop_app'), rapp_manager_srvs.StopApp)

        self.param = param
