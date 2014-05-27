#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
"""
.. module:: notifications

Simple class to help flag and manage notifications.
"""

##############################################################################
# Imports
##############################################################################

from .concert_client import ConcertClient

##############################################################################
# Classes
##############################################################################


class Notifications(object):
    """
    Convenience object that emulates a dictionary for flagging notifications in
    the update process. Keys are concert client states and values are booleans
    representing the notification flag for each state (typically only one is
    flagged at a time).
    """

    def __init__(self):
        self.flags = {}
        """
        The notification flag dictionary, keys are concert_msgs.ConcertClientState
        string constants and values are booleans.
        """
        self.reset_flags()

    def reset_flags(self):
        """
        Set flags for each state to false.
        """
        for state in ConcertClient.complete_list_of_states():
            self.flags[state] = False

    def __getitem__(self, state):
        """
        :param str state: state field for the notification (see concert_msgs/ConcertClientState.msg for permitted values)

        :returns: value of the flag for this state.
        :raises: :exc:`KeyError` if no such request
        :raises: :exc:`TypeError` if input state string is not valid
        """
        if state not in self.flags.keys():
            raise TypeError("invalid state %s" % state)
        return self.flags[state]

    def __setitem__(self, state, value):
        if state not in self.flags.keys():
            raise TypeError("invalid state %s" % state)
        self.flags[state] = value

    def is_flagged(self):
        """
        Looks to see if any of the fields have been flagged.

        :returns: true if a single field or more is true.
        :rtype: boolean
        """
        for value in self.flags.values():
            if value:
                return True
        return False
