#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
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

class Simple(object):
    """
    Defines a simple transition which just changes the state flag on the
    concert client without affecting anything else. This is the most
    common transition in the table.
    """

    def __init__(self, concert_client):
        self.concert_client = concert_client

    def __call__(self):
        """
        :param new_state str: representing a concert client state (strings from concert_msgs.ConcertClientState)
        """
        pass


class PendingToIdle(object):

    def __init__(self, concert_client):
        self.concert_client = concert_client

    def __call__(self, platform_info, rapps):
        """
        :param new_state str: representing a concert client state (strings from concert_msgs.ConcertClientState)
        """
        # this is legacy, and I think it's broken - I use concert alias now
        # self.msg.name = rocon_uri.parse(platform_info.uri).name.string
        self.concert_client.msg.platform_info = platform_info
        self.concert_client.msg.rapps = rapps

##############################################################################
# Transition Table
##############################################################################

StateTransitionTable = {
    (State.PENDING, State.BAD)       : Simple, #@IgnorePep8 noqa
    (State.PENDING, State.BLOCKING)  : Simple,
    (State.PENDING, State.BUSY)      : Simple,
    (State.PENDING, State.UNINVITED) : PendingToIdle,
    (State.PENDING, State.GONE)      : Simple,

    (State.UNINVITED, State.BAD)      : Simple,
    (State.UNINVITED, State.BLOCKING) : Simple,
    (State.UNINVITED, State.BUSY)     : Simple,
    (State.UNINVITED, State.JOINING)  : Simple,
    (State.UNINVITED, State.GONE)     : Simple,

    (State.JOINING, State.BAD)        : Simple,
    (State.JOINING, State.AVAILABLE)  : Simple,
    (State.JOINING, State.GONE)       : Simple,

    (State.AVAILABLE, State.BAD)      : Simple,
    (State.AVAILABLE, State.MISSING)  : Simple,
    (State.AVAILABLE, State.UNINVITED): Simple,
    (State.AVAILABLE, State.GONE)     : Simple,

    (State.MISSING, State.AVAILABLE)  : Simple,
    (State.MISSING, State.GONE)       : Simple,

    (State.BUSY, State.UNINVITED)     : Simple,
    (State.BUSY, State.GONE)          : Simple,

    (State.BLOCKING, State.GONE)      : Simple,

    (State.BAD, State.GONE)           : Simple,
}
