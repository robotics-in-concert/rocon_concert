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
    Convenience object for flagging notifications in the update process.
    """

    def __init__(self):
        self.flags = {}
        self.reset_flags()

    def reset_flags(self):
        for state in ConcertClient.complete_list_of_states():
            self.flags[state] = False

    def __getitem__(self, state):
        """
        :param state str: state field for the notification (see concert_msgs/ConcertClientState.msg for permitted values)

        :returns: named item.
        :raises: :exc:`KeyError` if no such request
        :raises: :exc:`TypeError` if no such request
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
