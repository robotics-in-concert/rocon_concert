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
import rocon_std_msgs.msg as rocon_std_msgs

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


class PendingToAvailable(object):
    """
    Triggered when the rocon_uri for this client has been successfully found flipped over
    into the concert.
    """
    def __init__(self, concert_client):
        self.concert_client = concert_client

    def __call__(self, rocon_uri):
        """
        :param rocon_uri: retrieved information about this client
        """
        platform_info = rocon_std_msgs.PlatformInfo()
        platform_info.uri = rocon_uri
        self.concert_client.msg.platform_info = platform_info


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
    (State.PENDING, State.BAD)       : Dummy,                #@IgnorePep8 noqa
    (State.PENDING, State.AVAILABLE) : PendingToAvailable,   #@IgnorePep8 noqa
    (State.PENDING, State.GONE)      : TransitionToGone,     #@IgnorePep8 noqa

    (State.JOINING, State.AVAILABLE)  : Dummy,              #@IgnorePep8 noqa
    (State.JOINING, State.GONE)       : TransitionToGone,   #@IgnorePep8 noqa

#   (State.AVAILABLE, State.BAD)      : Dummy,              #@IgnorePep8 noqa
    (State.AVAILABLE, State.MISSING)  : AvailableToMissing, #@IgnorePep8 noqa 
    (State.AVAILABLE, State.GONE)     : TransitionToGone,   #@IgnorePep8 noqa

    (State.MISSING, State.AVAILABLE)  : Dummy,              #@IgnorePep8 noqa
    (State.MISSING, State.GONE)       : TransitionToGone,   #@IgnorePep8 noqa

    (State.BLOCKING, State.GONE)      : TransitionToGone,   #@IgnorePep8 noqa

    (State.BAD, State.GONE)           : TransitionToGone,   #@IgnorePep8 noqa
}
"""
Table of valid transitions and their transition handlers.
"""
