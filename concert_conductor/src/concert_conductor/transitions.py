#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
"""
.. module:: transitions

This module does transition handling for the concert client state machine.
"""

##############################################################################
# Imports
##############################################################################

import concert_msgs.msg as concert_msgs

##############################################################################
# Aliases
##############################################################################

State = concert_msgs.ConcertClientState

##############################################################################
# Transitions
##############################################################################


class Dummy(object):
    """
    Dummy transition handler for when there is nothing to do.
    """

    def __init__(self, concert_client):
        self.concert_client = concert_client

    def __call__(self):
        """
        Nothing to do here.
        """
        pass


class TransitionToGone(object):
    """
    Transition handler when moving from any state to the gone state. This will always
    occur if the remote gateway has disappeared from the hub's awareness (happens
    when the remote gateway shuts down) or has been missing too long. We manually
    update the fact that the gateway is no longer available in the concert client's
    data here.
    """

    def __init__(self, concert_client):
        self.concert_client = concert_client

    def __call__(self, local_gateway):
        """
        Nothing to do here.
        """
        self.concert_client.msg.conn_stats.gateway_available = False
        local_gateway.request_pulls(self.concert_client.msg.gateway_name, cancel=True)


class PendingToUninvited(object):
    """
    Triggered when information about this client has been gathered.
    This information is relayed to the concert client object itself in this transition.
    """
    def __init__(self, concert_client):
        self.concert_client = concert_client

    def __call__(self, platform_info, rapps):
        """
        :param platform_info rocon_std_msgs/PlatformInfo: retrieved information about this client
        :param rapps rocon_app_manager_msgs/Rapp[]: list of rapps runnable by this client.
        """
        # this is legacy, and I think it's broken - I use concert alias now
        # self.msg.name = rocon_uri.parse(platform_info.uri).name.string
        self.concert_client.msg.platform_info = platform_info
        self.concert_client.msg.rapps = rapps


class AvailableToMissing(object):
    """
    Triggered when a robot is still with the concert, but has dropped its connection.
    """
    def __init__(self, concert_client):
        self.concert_client = concert_client

    def __call__(self):
        # Not implemented yet, thought I'd need to update something here,
        # but may actually not be necessary..
        pass

##############################################################################
# Transition Table
##############################################################################

StateTransitionTable = {
    (State.PENDING, State.BAD)       : Dummy, #@IgnorePep8 noqa
#    (State.PENDING, State.BLOCKING)  : Dummy,
#    (State.PENDING, State.BUSY)      : Dummy,
    (State.PENDING, State.UNINVITED) : PendingToUninvited,
    (State.PENDING, State.GONE)      : TransitionToGone,

    (State.UNINVITED, State.BAD)      : Dummy,
    (State.UNINVITED, State.BLOCKING) : Dummy,
    (State.UNINVITED, State.BUSY)     : Dummy,
    (State.UNINVITED, State.JOINING)  : Dummy,
    (State.UNINVITED, State.GONE)     : TransitionToGone,

#    (State.JOINING, State.BAD)        : Dummy,
    (State.JOINING, State.AVAILABLE)  : Dummy,
    (State.JOINING, State.GONE)       : TransitionToGone,

#    (State.AVAILABLE, State.BAD)      : Dummy,
    (State.AVAILABLE, State.MISSING)  : AvailableToMissing,
    (State.AVAILABLE, State.UNINVITED): Dummy,
    (State.AVAILABLE, State.GONE)     : TransitionToGone,

    (State.MISSING, State.AVAILABLE)  : Dummy,
    (State.MISSING, State.GONE)       : TransitionToGone,

    (State.BUSY, State.PENDING)       : Dummy,
    (State.BUSY, State.GONE)          : TransitionToGone,

    (State.BLOCKING, State.GONE)      : TransitionToGone,

    (State.BAD, State.GONE)           : TransitionToGone,
}
"""
Table of valid transitions and their transition handlers.
"""
