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

# def Simple(concert_client, new_state):
#     '''
#     Defines a simple transition which just changes the state flag on the
#     concert client without affecting anything else. This is the most
#     common transition in the table.
# 
#     :param concert_client ConcertClient: the concert client that is transitioning state
#     :param new_state str: representing a concert client state (strings from concert_msgs.ConcertClientState)
#     '''
#     old_state = concert_client.state
#     concert_client.state = new_state
#     rospy.loginfo("Conductor : concert client transition [%s->%s][%s]" % (old_state, new_state, concert_client.concert_alias))


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
        self.concert_client.msg.apps = rapps

# def transition_pending_to_idle(concert_client, platform_info, rapps):
#     """
#     :param platform_info rocon_std_msgs.PlatformInfo: platform information for this client.
#     :param rapps rocon_app_manager_msgs.App[]: list of apps runnable by the concert client.
#     """
#     # this is legacy, and I think it's broken - I use concert alias now
#     # self.msg.name = rocon_uri.parse(platform_info.uri).name.string
#     concert_client.msg.platform_info = platform_info
#     concert_client.msg.apps = rapps
#     concert_client.state = State.IDLE


##############################################################################
# Transition Table
##############################################################################

StateTransitionTable = {
    (State.PENDING, State.BAD)       : Simple, #@IgnorePep8 noqa
    (State.PENDING, State.BLOCKING)  : Simple,
    (State.PENDING, State.BUSY)      : Simple,
    (State.PENDING, State.IDLE)      : PendingToIdle,
    (State.PENDING, State.GONE)      : Simple,

    (State.IDLE, State.BAD)          : Simple,
    (State.IDLE, State.BLOCKING)     : Simple,
    (State.IDLE, State.BUSY)         : Simple,
    (State.IDLE, State.JOINING)      : Simple,
    (State.IDLE, State.GONE)         : Simple,

    (State.JOINING, State.BAD)       : Simple,
    (State.JOINING, State.AVAILABLE) : Simple,
    (State.JOINING, State.GONE)      : Simple,

    (State.AVAILABLE, State.BAD)     : Simple,
    (State.AVAILABLE, State.MISSING) : Simple,
    (State.AVAILABLE, State.IDLE)    : Simple,
    (State.AVAILABLE, State.GONE)    : Simple,

    (State.MISSING, State.AVAILABLE) : Simple,
    (State.MISSING, State.GONE)      : Simple,

    (State.BUSY, State.IDLE)         : Simple,
    (State.BUSY, State.GONE)         : Simple,

    (State.BLOCKING, State.GONE)     : Simple,

    (State.BAD, State.GONE)          : Simple,
}
