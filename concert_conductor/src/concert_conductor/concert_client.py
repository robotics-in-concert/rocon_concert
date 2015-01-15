#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
"""
.. module:: concert_client

This module wraps the concert_msgs data structure for a concert client in a
python class for convenient handling inside the concert conductor node.
"""

##############################################################################
# Imports
##############################################################################

import copy
import threading

import concert_msgs.msg as concert_msgs
import rocon_app_manager_msgs.msg as rapp_manager_msgs
import rocon_console.console as console
import rocon_uri
import rospy

from .exceptions import InvalidTransitionException
from . import transitions

##############################################################################
# Client Class
##############################################################################


class ConcertClient(object):
    """
    Envelops the concert client msg data structure that is published
    to the rest of the concert with a few extra fields and methods useful
    for management by the concert conductor class.

    .. seealso:: :class:`.ConcertConductor`, :class:`.ConcertClients`
    """
    __slots__ = [
        'msg',                 # concert_msgs.ConcertClient
        '_cached_status_msg',  # rocon_app_manager_msgs.Status, details of the last status update from the client
        'gateway_info',        # gateway_msgs.RemoteGateway
        '_timestamps',         # last observed and last state change timestamps
        '_transition_handlers',
        '_lock',               # for protecting access to the msg variable
    ]

    State = concert_msgs.ConcertClientState

    ##############################################################################
    # Construction and Destruction
    ##############################################################################

    def __init__(self, gateway_info, concert_alias, is_local_client=False):
        '''
          Initialise, finally ending with a call on the service to the client's
          platform info service.

          :param gateway_msgs.RemoteGateway gateway_info: information from this client's remote gateway
          :param str concert_alias: a string (+ index) human consumable form of the name
          :param bool is_local_client: is on the same ip as the concert (if we care)
        '''
        self.msg = concert_msgs.ConcertClient()
        """The publishable data structure describing a concert client."""
        self.msg.name = concert_alias
        self.msg.gateway_name = gateway_info.name
        self.msg.state = ConcertClient.State.PENDING
        self.msg.ip = gateway_info.ip
        self.msg.is_local_client = is_local_client
        self.gateway_info = gateway_info
        """Information about this client's gateway used for flipping, pulling and collecting connectivity statistics."""
        # This will get assigned when a msg comes in and set back to none once it is processed.
        # We only ever do processing on self.msg in one place to avoid threading problems
        # (the transition handlers) so that is why we store a cached copy here.
        self._cached_status_msg = None
        self._lock = threading.Lock()

        # timestamps
        self._timestamps = {}
        self._timestamps['last_seen'] = rospy.get_rostime()
        self._timestamps['last_state_change'] = rospy.get_rostime()

        # status
        rospy.Subscriber('/' + self.gateway_name.lower().replace(' ', '_') + '/' + 'status', rapp_manager_msgs.Status, self._ros_status_cb)

    ##############################################################################
    # Conveniences
    ##############################################################################

    @property
    def concert_alias(self):
        """The human readable concert alias for this client."""
        return self.msg.name

    @concert_alias.setter
    def concert_alias(self, value):
        self.msg.name = value

    @property
    def gateway_name(self):
        """The concert client's name on the gateway network (typically has postfixed uuid)"""
        return self.msg.gateway_name

    @gateway_name.setter
    def gateway_name(self, value):
        self.msg.gateway_name = value

    @property
    def is_local_client(self):
        """Whether it is a local client or not"""
        return self.msg.is_local_client

    @is_local_client.setter
    def is_local_client(self, value):
        self.msg.is_local_client = value

    @property
    def state(self):
        """The concert client's state (e.g. BUSY, AVAILABLE, MISSING, ...)"""
        return self.msg.state

    @state.setter
    def state(self, value):
        self.msg.state = value

    @property
    def platform_info(self):
        """Platform information about this concert client (icon, rocon uri, ...)"""
        return self.msg.platform_info

    @platform_info.setter
    def platform_info(self, value):
        self.msg.platform_info = value

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
    # Updating
    ##############################################################################

    def transition(self, new_state):
        '''
        Check that the transition is valid and then trigger the appropriate
        handle for making the transition.

        :param str new_state:
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

    def update(self, remote_gateway_info):
        """
        Common updates for clients that have been observed on the gateway network.

        This handles updates from both the incoming gateway information (remote_gateway_info)
        as well as the cached rapp manager information (self._cached_status_msg). Updates primarily
        go into the concert client message data (self.msg).

        :param gateway_msgs.RemoteGateway remote_gateway_info: latest message providing up to date connectivity information about this concert client.
        :returns: success or failure of the update
        :rtype: bool
        """
        self.touch()
        # remember, don't flag connection stats as worthy of a change.
        self.msg.conn_stats = remote_gateway_info.conn_stats
        #self.msg.last_connection_timestamp = rospy.Time.now()  # do we really need this?

        # don't update every client, just the ones that we need information from
        important_state = (self.state == ConcertClient.State.AVAILABLE) or (self.state == ConcertClient.State.UNINVITED) or (self.state == ConcertClient.State.MISSING)
        if self._cached_status_msg is not None and important_state:
            with self._lock:
                status_msg = copy.deepcopy(self._cached_status_msg)
                self._cached_status_msg = None
            # uri update
            uri = rocon_uri.parse(self.msg.platform_info.uri)
            uri.rapp = status_msg.rapp.name if status_msg.rapp_status == rapp_manager_msgs.Status.RAPP_RUNNING else ''
            self.msg.platform_info.uri = str(uri)
            return True  # something changed
        return False

    ##############################################################################
    # Utility
    ##############################################################################

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

          :param concert_msgs.ConcertClient msg: concert client information
          :param str indent: prefix each line with this string (usually for indentation so is just a number of spaces)
          :param bool show_state: include formatting of the client's state (increases verbosity).

          :returns: the formatted string representation of the object
          :rtype: str
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
    # Ros Callbacks
    ##############################################################################

    def _ros_status_cb(self, msg):
        """
        Update the concert client msg data with fields from this updated status.
        Just store it, ready to be processed in the update() method by the
        conductor spin loop (via the transition handlers)

        :param rocon_app_manager_msgs.Status msg:
        """
        with self._lock:
            self._cached_status_msg = msg
