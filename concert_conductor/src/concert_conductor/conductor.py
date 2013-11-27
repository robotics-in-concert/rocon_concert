#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import concert_msgs.msg as concert_msgs
import gateway_msgs.msg as gateway_msgs
import rocon_app_manager_msgs.msg as rapp_manager_msgs
import gateway_msgs.srv as gateway_srvs
import std_srvs.srv as std_srvs
import rocon_utilities

# local imports
from .concert_client import ConcertClient, ConcertClientException
from .ros_parameters import setup_ros_parameters

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
        # service clients
        self._remote_gateway_info_service = rospy.ServiceProxy("~remote_gateway_info", gateway_srvs.RemoteGatewayInfo)
        try:
            self._remote_gateway_info_service.wait_for_service()
        except rospy.ServiceException, e:
            raise e
        rospy.on_shutdown(self._shutdown)

        ##################################
        # Variables
        ##################################
        # Keys are client human friendly names, values the client class themselves
        self._concert_clients = {}  # Dict of name : conductor.ConcertClient, both visible and out of range
        self._invited_clients = {}  # Clients that have previously been invited, but disappeared
        # List of gateway names identifying bad clients
        self._bad_clients = []  # Used to remember clients that are bad.so we don't try and pull them again
        self._concert_name = None
        self._watcher_period = 1.0  # Period for the watcher thread (i.e. update rate)

        ##################################
        # Initialisation
        ##################################
        self._get_concert_name()
        self._param = setup_ros_parameters()
        self._publish_discovered_concert_clients()  # Publish an empty list, to latch it and start

    def batch_invite(self, concert_name, client_names):
        '''
          Batch invite a list of concert clients. Note we are not ever cancelling the invitations
          with this batch command.

          @param concert_name : pass our concert name in to the invitation
          @type string

          @param client_names : list of names of concert clients to invite
          @type string[]
        '''
        for name in client_names:
            try:
                # quiet abort checks
                if self._param['local_clients_only'] and not self._concert_clients[name].is_local_client:
                    continue
                # on with the invitation - rapp_manager_srvs.InviteResponse
                if self._concert_clients[name].invite(concert_name, name, cancel=False):
                    self._invited_clients[name] = True
            except KeyError:  # raised when name is not in the self._concert_clients keys
                rospy.logerr("Conductor : tried to invite unknown concert client [%s]" % name)

    def spin(self):
        '''
          Maintains the clientele list. We have to manage for two kinds here. Currently I use the same
          class interface for both with just a flag to differentiate, but it could probably use a split somewhere in the future.
        '''
        while not rospy.is_shutdown():
            # Grep list of remote clients from gateway
            # Grep list of local clients.
            # Prune unavailable clients.

            # For each new clients
            #   Resolve human friendly index
            #   add into the concert client list. Mark whether it is local client or not
            #   Invite the client if it was invited previously
            # If auto_invite is true
            #   Invite all available clients
            # If there is a change in the list, update the topic
            gateway_clients = self._get_gateway_clients()  # list of clients identified by gateway hash names
            number_of_pruned_clients = self._prune_client_list(gateway_clients)
            number_of_new_clients = 0
            new_clients = [c for c in gateway_clients if (c not in [client.gateway_name for client in self._concert_clients.values()])
                                                     and (c not in self._bad_clients)]
            # Create new clients info instance
            for gateway_hash_name in new_clients:
                gateway_name = rocon_utilities.gateway_basename(gateway_hash_name)
                try:
                    # remove the 16 byte hex hash from the name
                    same_name_count = 0
                    human_friendly_indices = set([])
                    for client in self._concert_clients.values():
                        if gateway_name == rocon_utilities.gateway_basename(client.gateway_name):
                            index = client.name.replace(gateway_name, "")
                            if index == "":
                                human_friendly_indices.add("0")
                            else:
                                human_friendly_indices.add(index)
                            same_name_count += 1
                    if same_name_count == 0:
                        concert_name = gateway_name
                    else:
                        human_friendly_index = -1
                        while True:
                            human_friendly_index += 1
                            if not str(human_friendly_index) in human_friendly_indices:
                                break
                        concert_name = gateway_name if human_friendly_index == 0 else gateway_name + str(human_friendly_index)
                    self._concert_clients[concert_name] = ConcertClient(concert_name, gateway_hash_name, is_local_client=self._is_local_client(gateway_hash_name))
                    rospy.loginfo("Conductor : new client found [%s]" % concert_name)
                    number_of_new_clients += 1

                    # re-invitation of clients that disappeared and came back
                    if concert_name in self._invited_clients:
                        self.batch_invite(self._concert_name, [concert_name])
                except rospy.exceptions.ROSInterruptException:  # ros is shutting down, ignore
                    break
                except Exception as e:
                    self._bad_clients.append(gateway_name)
                    rospy.loginfo("Conductor : failed to establish client [%s][%s][%s]" % (str(gateway_hash_name), str(e), type(e)))
            if self._param['auto_invite']:
                client_list = [client for client in self._concert_clients
                                     if (client not in self._invited_clients)
                                     or (client in self._invited_clients and self._invited_clients[client] == False)]
                if self._param['local_clients_only']:
                    client_list = [client for client in client_list if self._concert_clients[client].is_local_client == True]
                self.batch_invite(self._concert_name, client_list)
            # Continually publish so it goes to web apps for now (inefficient).
            self._publish_discovered_concert_clients(self.publishers["spammy_list_concert_clients"])
            # Long term solution
            if number_of_pruned_clients != 0 or number_of_new_clients != 0:
                self._publish_discovered_concert_clients()
            rospy.rostime.wallsleep(self._watcher_period)  # human time

    def _shutdown(self):
        """
            Last thing to do as a concert is shutting down - send an uninvite
            to all concert clients which will stop any apps currently running.

            This is usually done as a rospy shutdown hook.
        """
        # Don't worry about forcing the spin loop to come to a closure - rospy basically puts a halt
        # on it at the rospy.rostime call once we enter the twilight zone (shutdown hook period).
        for client_name, client in self._concert_clients.iteritems():
            client.invite(self._concert_name, client_name, cancel=True)
        try:
            rospy.loginfo("Conductor : sending shutdown request [gateway]")
            unused_response = rospy.ServiceProxy(concert_msgs.Strings.GATEWAY_SHUTDOWN, std_srvs.Empty)()
            rospy.loginfo("Conductor : sending shutdown request [hub]")
            unused_response = rospy.ServiceProxy(concert_msgs.Strings.HUB_SHUTDOWN, std_srvs.Empty)()
        except rospy.ServiceException as e:
            rospy.logerr("failed to externally shut down gateway/hub [%s]" % e)

    ###########################################################################
    # Helpers
    ###########################################################################

    def _is_local_client(self, gateway_name):
        '''
          Determine whether it is a local client (same machine) or remote

          @return true if it is a local client, false otherwise.
          @rtype Bool
        '''
        remote_gateway_info_request = gateway_srvs.RemoteGatewayInfoRequest()
        remote_gateway_info_request.gateways = []
        remote_gateway_info_response = self._remote_gateway_info_service(remote_gateway_info_request)
        remote_target_name = gateway_name
        remote_target_ip = None
        for gateway in remote_gateway_info_response.gateways:
            if gateway.name == remote_target_name:
                remote_target_ip = gateway.ip
                break
        if remote_target_ip is not None and self._concert_ip == remote_target_ip:
            return True
        else:
            return False

    def _get_gateway_clients(self):
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
        try:
            remote_gateway_info = self._remote_gateway_info_service()
            for remote_gateway in remote_gateway_info.gateways:
                for rule in remote_gateway.public_interface:
                    if rule.name.endswith(suffix):
                        visible_clients.append(rule.name[1:-(len(suffix) + 1)])  # escape the initial '/' and the trailing '/platform_info'
        except rospy.exceptions.ROSInterruptException:  # ros shutdown
            pass
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
            Provide a list of currently discovered clients. This gets called to provide
            input to both a latched publisher for state change updates as well as a periodic
            publisher to provide continuous updates with connection statistics.

            Currently we just publish invited clients - could also publish busy or blocking
            clients, but we don't have a use case yet for that information.

            @param list_concert_clients_publisher : the publisher to use (otherwise the default latched publisher)
            @type rospy.Publisher
        '''
        discovered_concert = concert_msgs.ConcertClients()
        for unused_client_name, client in self._concert_clients.iteritems():
            try:
                if client.is_invited:
                    discovered_concert.clients.append(client.to_msg_format())
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
                    self._concert_ip = gateway_info.ip
                    break
                else:
                    rospy.loginfo("Conductor : no hub yet available, spinning...")
            rospy.rostime.wallsleep(1.0)  # human time
