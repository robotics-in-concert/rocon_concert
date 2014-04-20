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
import rocon_app_manager_msgs.msg as rapp_manager_msgs
import rocon_app_manager_msgs.srv as rapp_manager_srvs
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_std_msgs.srv as rocon_std_srvs
import rocon_console.console as console
import rosservice

from .exceptions import ConcertClientException
from .exceptions import InvalidTransitionException
from . import transitions

##############################################################################
# Client Class
##############################################################################


class ConcertClient(object):
    __slots__ = [
        'msg',  # concert_msgs.ConcertClient
        'gateway_info',  # gateway_msgs.RemoteGateway
        '_timestamps',    # last observed and last state change timestamps
        '_transition_handlers',
    ]
    # convenience aliases to self.msg.xxx (see bottom of this module)
    #   concert_alias/name, gateway_name, state, is_local_client and platform_info

    State = concert_msgs.ConcertClientState

    ##############################################################################
    # Construction and Destruction
    ##############################################################################

    def __init__(self, gateway_info, concert_alias, is_local_client=False):
        '''
          Initialise, finally ending with a call on the service to the client's
          platform info service.

          :param gateway_info gateway_msgs.RemoteGateway: information from this client's remote gateway
          :param concert_alias str: a string (+ index) human consumable form of the name
          :param is_local_client bool: is on the same ip as the concert (if we care)
        '''
        self.msg = concert_msgs.ConcertClient()
        self.msg.name = concert_alias
        self.msg.gateway_name = gateway_info.name
        self.msg.state = ConcertClient.State.PENDING
        self.msg.is_local_client = is_local_client
        self.gateway_info = gateway_info

        # timestamps
        self._timestamps = {}
        self._timestamps['last_seen'] = rospy.get_rostime()
        self._timestamps['last_state_change'] = rospy.get_rostime()

    def _setup_services(self):
        services = {}
        services['status'] = rospy.ServiceProxy('/' + str(self.gateway_name + '/' + 'status'), rapp_manager_srvs.Status, persistent=True)
        return services

    ##############################################################################
    # Timestamping
    ##############################################################################

    def touch(self):
        """
        The timestamp is updated whenever this concert client is visible on the gateway network.
        This is the api that updates the timestamp.
        """
        self._timestamps['last_seen'] = rospy.get_rostime()

    def time_since_last_seen(self):
        """
        :returns: the time since the client was last observed on the gateway network
        :rtype: float
        """
        current_time = rospy.get_rostime()
        difference = current_time - self._timestamps['last_seen']
        return difference.to_sec()

    def time_since_last_state_change(self):
        """
        This is useful to trigger some timeouts on state changes from a higher level.

        :returns: the time since the client last changed state
        :rtype: float
        """
        current_time = rospy.get_rostime()
        difference = current_time - self._timestamps['last_state_change']
        return difference.to_sec()

    ##############################################################################
    # Transition Handlers
    ##############################################################################

    def transition(self, new_state):
        '''
        Check that the transition is valid and then trigger the appropriate
        handle for making the transition.

        :param new_state str:
        :returns: function handler for the transition (varying args)
        :raises: :exc:`.InvalidTransitionException` if the transition is not permitted
        '''
        old_state = self.state
        if (old_state, new_state) in transitions.StateTransitionTable.keys():
            rospy.loginfo("Conductor : concert client transition [%s->%s][%s]" % (old_state, new_state, self.concert_alias))
            self._timestamps['last_state_change'] = rospy.get_rostime()
            self.state = new_state
            transition_handler = transitions.StateTransitionTable[(old_state, new_state)](self)
            return transition_handler.__call__
        else:
            rospy.logerr("Conductor : invalid concert client transition [%s->%s][%s]" % (self.state, new_state, self.concert_alias))
            raise InvalidTransitionException("invalid concert client transition [%s->%s][%s]" % (old_state, new_state, self.concert_alias))

    ##############################################################################
    # Abstract States
    ##############################################################################

    def is_invited(self):
        return self.state == ConcertClient.State.AVAILABLE or self.state == ConcertClient.State.MISSING

    def is_unavailable(self):
        return self.state == ConcertClient.State.BAD or self.state == ConcertClient.State.BLOCKING or self.state == ConcertClient.State.BUSY

    ##############################################################################
    # Utility
    ##############################################################################

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
        return self.msg

    @staticmethod
    def complete_list_of_states():
        """
        Gets the list of all possible states that a concert client may be in.

        :returns: list of states from concert_msgs/ConcertClientState
        :rtype: str[]
        """
        # funny way of getting all the states that are defined in ConcertClientState.msg
        return concert_msgs.ConductorGraph.__slots__

    ##############################################################################
    # Printing
    ##############################################################################

    def __str__(self):
        '''
          Format the client into a human-readable string.
        '''
        s = ''
        s += console.green + "  %s" % self.msg.name + console.reset + '\n'
        s += console.cyan + "    Gateway Name" + console.reset + " : " + console.yellow + "%s" % self.msg.name + console.reset + '\n'  # noqa
        s += console.cyan + "    Is Local" + console.reset + "     : " + console.yellow + "%s" % self.msg.is_local_client + console.reset + '\n'  # noqa
        s += console.cyan + "    State" + console.reset + "        : " + console.yellow + "%s" % self.state.capitalize() + console.reset + '\n'  # noqa
        if self.state != ConcertClient.State.PENDING and self.state != ConcertClient.State.BAD:
            s += console.cyan + "    Rocon Uri" + console.reset + "    : " + console.yellow + "%s" % self.msg.platform_info.uri + console.reset + '\n'  # noqa
        return s

    @staticmethod
    def msg2string(msg, indent="", show_state=True):
        '''
          Format the client into a human-readable string.
          :param msg concert_msgs.ConcertClient: concert client information
        '''
        s = ''
        s += indent + "-" + console.cyan + " Concert Alias" + console.reset + ": " + console.green + "%s" % msg.name + console.reset + '\n'  # noqa
        s += indent + console.cyan + "  Gateway Name" + console.reset + " : " + console.yellow + "%s" % msg.gateway_name + console.reset + '\n'  # noqa
        s += indent + console.cyan + "  Is Local" + console.reset + "     : " + console.yellow + "%s" % msg.is_local_client + console.reset + '\n'  # noqa
        if show_state:
            s += indent + console.cyan + "  State" + console.reset + "        : " + console.yellow + "%s" % msg.state.capitalize() + console.reset + '\n'  # noqa
        if msg.state != ConcertClient.State.PENDING and msg.state != ConcertClient.State.BAD:
            s += console.cyan + indent + "  Rocon Uri" + console.reset + "    : " + console.yellow + "%s" % msg.platform_info.uri + console.reset + '\n'  # noqa
        return s

    ##############################################################################
    # Conveniences
    ##############################################################################

    @property
    def concert_alias(self):
        return self.msg.name

    @concert_alias.setter
    def concert_alias(self, value):
        self.msg.name = value

    @property
    def gateway_name(self):
        return self.msg.gateway_name

    @gateway_name.setter
    def gateway_name(self, value):
        self.msg.gateway_name = value

    @property
    def is_local_client(self):
        return self.msg.is_local_client

    @is_local_client.setter
    def is_local_client(self, value):
        self.msg.is_local_client = value

    @property
    def state(self):
        return self.msg.state

    @state.setter
    def state(self, value):
        self.msg.state = value

    @property
    def platform_info(self):
        return self.msg.platform_info

    @platform_info.setter
    def platform_info(self, value):
        self.msg.platform_info = value

    ##############################################################################
    # Graveyard
    ##############################################################################

    def _update(self):
        '''
          Adds the current client status to the platform_info, list_apps information already
          retrieved from the client and puts them in a convenient msg format,
          ready for exposing outside the conductor (where? I can't remember).

          @raise rospy.service.ServiceException : when assumed service link is unavailable
        '''
#         try:
#             status = self._status_service(rapp_manager_srvs.StatusRequest())
#         except rospy.service.ServiceException:
#             raise ConcertClientException("client platform information services unavailable (disconnected?)")
#         #self.msg.name = status.namespace
# 
#         # Updating app status
#         self.msg.app_status = status.application_status
# 
#         self.msg.is_local_client = self.is_local_client
# 
#         self.msg.last_connection_timestamp = rospy.Time.now()
#         if status.remote_controller == rapp_manager_msgs.Constants.NO_REMOTE_CONNECTION:
#             self.msg.client_status = concert_msgs.Constants.CONCERT_CLIENT_STATUS_AVAILABLE
#         # Todo - fix this
#         #elif status.remote_controller == _this_concert_name:
#         #    self.msg.client_status = concert_msgs.Constants.CONCERT_CLIENT_STATUS_CONNECTED
#         else:
#             self.msg.client_status = concert_msgs.Constants.CONCERT_CLIENT_STATUS_CONNECTED
#         #    self.msg.client_status = concert_msgs.Constants.CONCERT_CLIENT_STATUS_UNAVAILABLE
# 
#         try:
#             remote_gateway_info = self._remote_gateway_info_service()
#         except rospy.service.ServiceException:
#             raise ConcertClientException("remote client statistics unavailable")
#         except rospy.ROSInterruptException:
#             raise ConcertClientException("remote client statistics unavailable, ros shutdown")
#         gateway_found = False
#         for gateway in remote_gateway_info.gateways:
#             if gateway.name == self.gateway_name:
#                 self.msg.conn_stats = gateway.conn_stats
#                 gateway_found = True
#                 break
#         if not gateway_found:
#             raise ConcertClientException("couldn't find remote gateway info while update client information")
