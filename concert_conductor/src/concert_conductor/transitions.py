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
    (State.PENDING, State.BLOCKING)  : Dummy,
    (State.PENDING, State.BUSY)      : Dummy,
    (State.PENDING, State.UNINVITED) : PendingToUninvited,
    (State.PENDING, State.GONE)      : Dummy,

    (State.UNINVITED, State.BAD)      : Dummy,
    (State.UNINVITED, State.BLOCKING) : Dummy,
    (State.UNINVITED, State.BUSY)     : Dummy,
    (State.UNINVITED, State.JOINING)  : Dummy,
    (State.UNINVITED, State.GONE)     : Dummy,

    (State.JOINING, State.BAD)        : Dummy,
    (State.JOINING, State.AVAILABLE)  : Dummy,
    (State.JOINING, State.GONE)       : Dummy,

    (State.AVAILABLE, State.BAD)      : Dummy,
    (State.AVAILABLE, State.MISSING)  : AvailableToMissing,
    (State.AVAILABLE, State.UNINVITED): Dummy,
    (State.AVAILABLE, State.GONE)     : Dummy,

    (State.MISSING, State.AVAILABLE)  : Dummy,
    (State.MISSING, State.GONE)       : Dummy,

    (State.BUSY, State.UNINVITED)     : Dummy,
    (State.BUSY, State.GONE)          : Dummy,

    (State.BLOCKING, State.GONE)      : Dummy,

    (State.BAD, State.GONE)           : Dummy,
}
