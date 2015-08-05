#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#

"""
.. module:: concert_clients

This module tracks all known concert clients monitored by the conductor.
"""

##############################################################################
# Imports
##############################################################################

import rocon_app_manager_msgs.msg as rocon_app_manager_msgs
import rocon_app_manager_msgs.srv as rocon_app_manager_srvs
import rocon_gateway_utils
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_std_msgs.srv as rocon_std_srvs
import rosgraph
import rospy

from .concert_client import ConcertClient
from .notifications import Notifications
from .transitions import State

##############################################################################
# Methods
##############################################################################


def is_concert_client_gateway(remote_gateway):
    """
    Currently just checks the remote gateways for 'platform_info'
    services ...which should be representative of a
    concert client (aye, not real safe like).

    :param remote_gateway gateway_msgs.RemoteGateway
    :return: whether or not it is a concert client gateway.
    :rtype: bool
    """
    for rule in remote_gateway.public_interface:
        if rule.name.endswith('platform_info'):
            return True
    return False


def _is_local_client(concert_ip, gateway_ip):
    '''
      Determine whether it is a gateway on the same ip as the concert.

      :param concert_ip str: ip used to check if a concert client is local or not.
      :param gateway_ip str: ip of the gateway to check
      :return: true or false depending on the result
      :rtype bool:
    '''
    if gateway_ip == 'localhost':
        return True
    if concert_ip == gateway_ip:
        return True
    if rosgraph.network.is_local_address(gateway_ip):
        return True
    return False


##############################################################################
# Class
##############################################################################


class ConcertClients(object):
    """
    Convenience class that emulates a dictionary holding :class:`.ConcertClient`
    objects keyed by their gateway names.

    .. seealso:: :class:`.ConcertClient`
    """

    __slots__ = [
        '_concert_name',      # concert name, used for inviting clients
        '_local_gateway',     # for interacting with the local gateway
        '_param',             # parameter configuration from the ros parameter server
        '_flat_client_dict',  # { gateway_name : conductor.ConcertClient }
        '_clients_by_state',  # super dictionary of all known concert clients keyed by state (see __init__)
        '_state_handlers',    # { State : handler function } for state machine handling of concert clients
        '_publish_concert_clients',
        '_publish_graph',
    ]

    ##############################################################################
    # Construction and Destruction
    ##############################################################################

    def __init__(self, local_gateway, parameters, publish_concert_clients, publish_graph_callback):
        """
        :param local_gateway: object used to interact with the local gateway (for pull requests etc)
        :param dict params: ros parameters available to the conductor.
        :param func publish_concert_clients: function that takes a _clients_by_state variable for concert clients publishing
        :param func publish_graph_callback: function callback that accepts an _all_client_dicts variable for ros or stdout publishing
        """
        self._concert_name = local_gateway.name
        self._local_gateway = local_gateway
        self._param = parameters
        self._publish_concert_clients = publish_concert_clients
        self._publish_graph = publish_graph_callback

        self._flat_client_dict = {}  # { remote gateway name : concert_client.ConcertClient }
        """
        Dictionary of all known concert clients whether they are invited,
        uninvited, bad, firewalled or otherwise.
        """
        self._clients_by_state = {}
        """
        Super dictionary of all known concert clients keyed into smaller dictionaries by state.
        """
        self._state_handlers = {}
        """
        Dictionary of all handlers responsible for looking after the state updates of a concert
        client.
        """
        for state in ConcertClient.complete_list_of_states():
            self._clients_by_state[state] = {}  # { remote gateway name : concert_client.ConcertClient }
            self._state_handlers[state] = getattr(self, "_update_" + state + "_client")

    def __contains__(self, gateway_name):
        return gateway_name in self._flat_client_dict

    def __getitem__(self, gateway_name):
        return self._flat_client_dict[gateway_name]

    def shutdown(self):
        """
        Uninvites all currently connected concert clients.
        """
        for concert_client in self._clients_by_state[State.AVAILABLE].values():
            self._uninvite_client(concert_client)

    ##############################################################################
    # Runtime
    ##############################################################################

    def update(self, visible_remote_gateway_list):
        """
        :param gateway_msgs.RemoteGateway[] visible_remote_gateway_list: list of the remote gateways visible on the concert hub.

        :return: whether a pertinent change occured in this concert client's list needs republishing (e.g. went missing...)
        :rtype: bool
        """
        # cut any non-concert gateways
        remote_gateway_index = {}
        for remote_gateway in visible_remote_gateway_list:  # gateway_msgs.RemoteGateway[]
            if is_concert_client_gateway(remote_gateway):
                remote_gateway_index[remote_gateway.name] = remote_gateway

        # set flags to look for notifications
        notifications = Notifications()

        # existing client updates
        for (gateway_name, concert_client) in self._flat_client_dict.items():
            if gateway_name in remote_gateway_index.keys():
                gateway_info = remote_gateway_index[gateway_name]  # gateway_msgs.RemoteGateway
                # common client update tasks - update the timestamp and check if it got a status update
                common_update_result = concert_client.update(gateway_info)  # this 'touches' the object and also checks if a rapp manager status message came in
                # now relay to one of the update_STATE_client handlers
                state_dependant_result = self._state_handlers[concert_client.state](gateway_info, concert_client)
                result = common_update_result or state_dependant_result
                del remote_gateway_index[gateway_name]  # remove it from the index.
            else:
                result = self._state_handlers[concert_client.state](None, concert_client)
            if result:
                notifications[concert_client.state] = True

        # new clients
        if remote_gateway_index:
            notifications[State.PENDING] = True
        for remote_gateway in remote_gateway_index.values():  # gateway_msgs.RemoteGateway[]
            self._create_new_client(remote_gateway)

        # Notifications if something changed
        if notifications.is_flagged():
            self._publish_graph(self._clients_by_state)
        if notifications[State.MISSING] or notifications[State.UNINVITED] or notifications[State.AVAILABLE]:
            self._publish_concert_clients(self._clients_by_state, changes_only=True)
        # Periodic publisher
        self._publish_concert_clients(self._clients_by_state, changes_only=False)

    ##############################################################################
    # God Handling (create, destroy) of concert clients
    ##############################################################################

    def _create_new_client(self, remote_gateway):
        """
        :param remote_gateway concert_msgs.RemoteGateway: information from the gateway network
        """
        rospy.loginfo("Conductor : new client discovered [%s]" % remote_gateway.name)
        concert_alias = self._generate_concert_alias(remote_gateway.name)
        self._local_gateway.request_pulls(remote_gateway.name)
        is_local_client = _is_local_client(self._local_gateway.ip, remote_gateway.ip)  # is it on the same machine as the concert
        concert_client = ConcertClient(remote_gateway, concert_alias, is_local_client)
        self._flat_client_dict[remote_gateway.name] = concert_client
        self._clients_by_state[State.PENDING][remote_gateway.name] = concert_client

    def _send_to_oblivion(self, gateway_name):
        self._local_gateway.request_pulls(gateway_name, cancel=True)  # cancel default pulls
        del self._flat_client_dict[gateway_name]
        for concert_clients in self._clients_by_state.values():
            try:
                del concert_clients[gateway_name]
            except KeyError:
                pass

    ##############################################################################
    # Concert Client State Machine Handlers
    ##############################################################################

    def _update_pending_client(self, remote_gateway, concert_client):
        """
        Does a quick check to see if platform_info and list_rapps services have landed in the concert
        from the client. If not, it either quickly exits, or changes to a BAD state if it's been
        too long in the pending state.

        If the services are found, it extracts the information and dumps that into the concert client
        instance before switching state to UNINVITED.

        :param concert_msgs.RemoteGateway remote_gateway: updated information from the gateway network
        :param concert_client.ConcertClient concert_client: update a client that isn't currently visible.
        :returns: notification of whether there was an update or not
        :rtype bool:
        """
        # it disappeared
        if remote_gateway is None:
            self._transition(concert_client, State.GONE)(self._local_gateway)

        # Check for handles
        platform_info_service_name = '/' + concert_client.gateway_name.lower().replace(' ', '_') + '/' + 'platform_info'
        list_rapps_service_name = '/' + concert_client.gateway_name.lower().replace(' ', '_') + '/' + 'list_rapps'
        try:
            rospy.wait_for_service(platform_info_service_name, self._param['service_timeout'])
            rospy.wait_for_service(list_rapps_service_name,self._param['service_timeout'])
        except rospy.ROSException:  # timeout
#            JL : I don't see any critical reason the client should be marked as BAD with timeout. It just makes problems if concert is running slow network
#                 See github rocon_concert #302 issue.
#            if concert_client.time_since_last_state_change() > 10.0:
#                rospy.logwarn("Conductor : timed out waiting for client's platform_info and list_rapps topics to be pulled [%s]" % concert_client.concert_alias)
#                self._transition(concert_client, State.BAD)()
#                self._local_gateway.request_pulls(remote_gateway.name, cancel=True)
#                return True
#            else:
            return False  # let's keep trying till the last_state_change timeout kicks in
        except rospy.ROSInterruptException:
            return False
        # Introspect the client
        platform_info_service = rospy.ServiceProxy(platform_info_service_name, rocon_std_srvs.GetPlatformInfo)
        list_rapps_service = rospy.ServiceProxy(list_rapps_service_name, rocon_app_manager_srvs.GetRappList)
        try:
            platform_info = platform_info_service().platform_info
            if platform_info.version != rocon_std_msgs.Strings.ROCON_VERSION:
                rospy.logwarn("Conductor : concert client and conductor rocon versions do not match [%s][%s]" % (platform_info.version, rocon_std_msgs.Strings.ROCON_VERSION))
                self._transition(concert_client, State.BAD)()
                self._local_gateway.request_pulls(remote_gateway.name, cancel=True)
                return True
            available_rapps = list_rapps_service().available_rapps
            self._transition(concert_client, State.UNINVITED)(platform_info, available_rapps)
            # no longer needed as we have the information stored
            self._local_gateway.request_pulls(remote_gateway.name, cancel=True, service_names=['platform_info', 'list_rapps'], topic_names=[])
            return True
        except (rospy.ServiceException, rospy.ROSInterruptException):
            return False  # let's keep trying till the last_state_change timeout kicks in
        return False

    def _update_bad_client(self, remote_gateway, concert_client):
        """
        :param remote_gateway concert_msgs.RemoteGateway: updated information from the gateway network
        :param concert_client concert_client.ConcertClient: update a client that isn't currently visible.
        :returns: notification of whether there was an update or not
        :rtype bool:
        """
        # it disappeared
        if remote_gateway is None:
            self._transition(concert_client, State.GONE)(self._local_gateway)
            return True
        return False

    def _update_blocking_client(self, remote_gateway, concert_client):
        """
        :param concert_msgs.RemoteGateway remote_gateway: updated information from the gateway network
        :param concert_client.ConcertClient concert_client: update a client that isn't currently visible.
        :returns: notification of whether there was an update or not
        :rtype bool:
        """
        # it disappeared
        if remote_gateway is None:
            self._transition(concert_client, State.GONE)(self._local_gateway)
            return True
        return False

    def _update_busy_client(self, remote_gateway, concert_client):
        """
        :param concert_msgs.RemoteGateway remote_gateway: updated information from the gateway network
        :param concert_client.ConcertClient concert_client: update a client that isn't currently visible.
        :returns: notification of whether there was an update or not
        :rtype bool:
        """
        # it disappeared
        if remote_gateway is None:
            self._transition(concert_client, State.GONE)(self._local_gateway)
            return True

        if not concert_client.is_controlled:
            self._transition(concert_client, State.UNINVITED)()
            return True
        return False

    def _update_uninvited_client(self, remote_gateway, concert_client):
        """
        Only handling automatic invitiations for now.

        :param concert_msgs.RemoteGateway remote_gateway: updated information from the gateway network
        :param concert_client.ConcertClient concert_client: update a client that isn't currently visible.
        :returns: notification of whether there was an update or not
        :rtype bool:
        """
        # it disappeared
        if remote_gateway is None:
            self._transition(concert_client, State.GONE)(self._local_gateway)
            self._local_gateway.request_pulls(remote_gateway.name, cancel=True, service_names=['invite'], topic_names=[])
            return True

        if self._param['local_clients_only'] and not concert_client.is_local_client:
            rospy.loginfo("Conductor : shunning this (non-local) client [%s][%s]" % (concert_client.concert_alias, concert_client.gateway_name))
            self._transition(concert_client, State.BLOCKING)()
            self._local_gateway.request_pulls(remote_gateway.name, cancel=True, service_names=['invite'], topic_names=[])
            return True
        elif self._param['auto_invite']:
            # try an invite
            invite = rospy.ServiceProxy('/' + concert_client.gateway_name.lower().replace(' ', '_') + '/invite', rocon_app_manager_srvs.Invite)
            try:
                response = invite(remote_target_name=self._concert_name,
                                  application_namespace=concert_client.concert_alias.lower().replace(' ', '_'),
                                  cancel=False
                                  )
                if response.result:
                    self._transition(concert_client, State.JOINING)()
                    return True
                else:
                    rospy.logwarn("Conductor : failed to invite client [%s][%s]" % (response.message, concert_client.gateway_name))
                    if (
                        response.error_code == rocon_app_manager_msgs.ErrorCodes.LOCAL_INVITATIONS_ONLY or
                        response.error_code == rocon_app_manager_msgs.ErrorCodes.INVITING_CONTROLLER_BLACKLISTED or
                        response.error_code == rocon_app_manager_msgs.ErrorCodes.INVITING_CONTROLLER_NOT_WHITELISTED
                       ):
                        rospy.logwarn("Conductor : invitation to %s was blocked [%s][%s]" % (concert_client.gateway_name, response.message, concert_client.gateway_name))
                        self._transition(concert_client, State.BLOCKING)()
                        self._local_gateway.request_pulls(remote_gateway.name, cancel=True, service_names=['invite'], topic_names=[])
                    elif response.error_code == rocon_app_manager_msgs.ErrorCodes.ALREADY_REMOTE_CONTROLLED:
                        rospy.logwarn("Conductor : invitation to %s was refused [%s][%s]" % (concert_client.gateway_name, response.message, concert_client.gateway_name))
                        self._transition(concert_client, State.BUSY)()
                    else:
                        rospy.logwarn("Conductor : invitation to %s failed [%s][%s]" % (concert_client.gateway_name, response.message, concert_client.gateway_name))
                        self._transition(concert_client, State.BAD)()
                        self._clients_by_state[State.BAD][remote_gateway.name] = concert_client
                        self._local_gateway.request_pulls(remote_gateway.name, cancel=True, service_names=['invite'], topic_names=[])
                    return True
            except rospy.ServiceException:
                rospy.logwarn("Conductor : invitation to %s was sent, but not received [service exception][%s]" % (concert_client.concert_alias, concert_client.gateway_name))
                self._transition(concert_client, State.BAD)()
                self._local_gateway.request_pulls(remote_gateway.name, cancel=True, service_names=['invite'], topic_names=[])
                return True
            except rospy.ROSInterruptException:  # interrupted by conductor's rosmaster shutdown
                return False
        else:  # not automatically inviting it
            # could actually check its status to see if its busy I suppose, but not worrying about that for now.
            pass
        return False

    def _update_joining_client(self, remote_gateway, concert_client):
        """
        :param concert_msgs.RemoteGateway remote_gateway: updated information from the gateway network
        :param concert_client.ConcertClient concert_client: update a client that isn't currently visible.
        :returns: notification of whether there was an update or not
        :rtype bool:
        """
        # it disappeared
        if remote_gateway is None:
            self._transition(concert_client, State.GONE)(self._local_gateway)
            return True

        # Check for handles
        start_app_service_name = '/' + concert_client.gateway_name.lower().replace(' ', '_') + '/start_rapp'
        stop_app_service_name = '/' + concert_client.gateway_name.lower().replace(' ', '_') + '/stop_rapp'
        try:
            rospy.wait_for_service(start_app_service_name, self._param['service_timeout'])
            rospy.wait_for_service(stop_app_service_name, self._param['service_timeout'])
        except rospy.ROSException:  # timeout
#            JL : I don't see any critical reason the client should be marked as BAD with timeout. It just makes problems if concert is running slow network
#                 See github rocon_concert #302 issue.
#
#            if concert_client.time_since_last_state_change() > 10.0:
#                rospy.logwarn("Conductor : timed out waiting for client's start_rapp and stop_rapp services to be flipped [%s]" % concert_client.concert_alias)
#                self._transition(concert_client, State.BAD)()
#                return True
#            else:
            return False  # let's keep trying till the last_state_change timeout kicks in
        except rospy.ROSInterruptException:
            return False
        # If we reach here, we've found the handles.
        self._transition(concert_client, State.AVAILABLE)()
        return True

    def _update_available_client(self, remote_gateway, concert_client):
        """
        :param concert_msgs.RemoteGateway remote_gateway: updated information from the gateway network
        :param concert_client.ConcertClient concert_client: update a client that isn't currently visible.
        :returns: notification of whether there was an update or not
        :rtype bool:
        """
        # it disappeared
        if remote_gateway is None:
            del self._flat_client_dict[concert_client.gateway_name]
            self._transition(concert_client, State.GONE)(self._local_gateway)
            return True
        if not remote_gateway.conn_stats.gateway_available:  # it's dropped off it's wireless
            self._transition(concert_client, State.MISSING)()
            return True
        return False

    def _update_missing_client(self, remote_gateway, concert_client):
        """
        :param remote_gateway concert_msgs.RemoteGateway: updated information from the gateway network
        :param concert_client concert_client.ConcertClient: update a client that isn't currently visible.
        :returns: notification of whether there was an update or not
        :rtype bool:
        """
        if remote_gateway is None:
            self._transition(concert_client, State.GONE)(self._local_gateway)
            return True
        if remote_gateway.conn_stats.gateway_available:  # it's dropped off it's wireless
            self._transition(concert_client, State.AVAILABLE)()
            return True
        return False

    def _update_gone_client(self, remote_gateway, concert_client):
        """
        :param remote_gateway concert_msgs.RemoteGateway: updated information from the gateway network
        :param concert_client concert_client.ConcertClient: update a client that has left the concert.
        :returns: notification of whether there was an update or not
        :rtype bool:
        """
        if concert_client.time_since_last_state_change() > self._param['oblivion_timeout']:
            return True
        return False

    ##############################################################################
    # Utilities
    ##############################################################################

    def _transition(self, concert_client, new_state):
        """
        Wrap some common code for transitioning a client.

        :param concert_clients.ConcertClient concert_client:
        :param new_state State:
        """
        old_state = concert_client.state
        self._clients_by_state[new_state][concert_client.gateway_name] = concert_client
        del self._clients_by_state[old_state][concert_client.gateway_name]
        return concert_client.transition(new_state)

    def _uninvite_client(self, concert_client):
        """
        Uninvite a client. For now, this is only done on shutdown and so we
        don't handle any errors yet.

        :param concert_clients.ConcertClient concert_client:
        """
        if concert_client.state != State.AVAILABLE:
            rospy.logwarn("Conductor : stubbornly refusing to uninvite an uninvited client [%s][%s]" % (concert_client.concert_alias, concert_client.gateway_name))
            return
        invite = rospy.ServiceProxy('/' + concert_client.gateway_name.lower().replace(' ', '_') + '/invite', rocon_app_manager_srvs.Invite)
        try:
            response = invite(remote_target_name=self._concert_name,
                              application_namespace=concert_client.concert_alias,
                              cancel=True
                              )
            if response.result:
                concert_client.transition(State.UNINVITED)()
                self._clients_by_state[State.UNINVITED][concert_client.gateway_name] = concert_client
                del self._clients_by_state[State.AVAILABLE][concert_client.gateway_name]
            else:
                rospy.logwarn("Conductor : failed to uninvite %s [%s][%s]" % (concert_client.concert_alias, response.message, concert_client.gateway_name))
                concert_client.transition(State.BAD)()
                self._clients_by_state[State.BAD][concert_client.gateway.name] = concert_client
                del self._clients_by_state[State.AVAILABLE][concert_client.gateway.name]
        except rospy.ServiceException:
            rospy.logwarn("Conductor : uninvite to %s was sent, but not received [service exception][%s]" % (concert_client.concert_alias, concert_client.gateway_name))
            concert_client.transition(State.BAD)()
            self._clients_by_state[State.BAD][concert_client.gateway_name] = concert_client
            del self._clients_by_state[State.AVAILABLE][concert_client.gateway_name]
        except rospy.ROSInterruptException:  # interrupted by conductor's rosmaster shutdown
            pass

    def _generate_concert_alias(self, gateway_name):
        '''
        Generate a friendly concert alias for this client given the (usually) uuid suffixed gateway name.

        :param str gateway_name: the uuid'd gateway name (e.g. kobuki95fbd06982344cfc9b013ef7b184e420)
        :return: the concert alias
        :rtype: str
        '''
        gateway_basename = rocon_gateway_utils.gateway_basename(gateway_name)
        # remove the 16 byte hex hash from the name
        same_name_count = 0
        human_friendly_indices = set([])
        for concert_client in self._flat_client_dict.values():
            if gateway_basename == rocon_gateway_utils.gateway_basename(concert_client.gateway_name):
                index = concert_client.concert_alias.replace(gateway_basename, "")
                if index == "":
                    human_friendly_indices.add("0")
                else:
                    human_friendly_indices.add(index)
                same_name_count += 1

        human_friendly_index = -1
        while True:
            human_friendly_index += 1
            if not str(human_friendly_index) in human_friendly_indices:
                break
        concert_name = gateway_basename if human_friendly_index == 0 else gateway_basename + str(human_friendly_index)
        return concert_name
