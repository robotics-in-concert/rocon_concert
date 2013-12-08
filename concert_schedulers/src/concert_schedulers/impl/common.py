#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

# local imports
import utils

##############################################################################
# Classes
##############################################################################


class ConcertClient(object):
    __slots__ = [
            'msg',          # concert_msgs/ConcertClient
            'name',         # alias to msg.name
            'allocated',    # boolean value representing whether it has been allocated or not.
            '_request_id',  # id (uuid hex string) of the request it is allocated to
            '_resource'     # scheduler_msgs.Resource it fulfills
        ]

    def __init__(self, msg):
        self.msg = msg
        self.allocated = False
        self._request_id = None
        self._resource = None

        # aliases
        self.name = self.msg.name

    def allocate(self, request_id, resource):
        self.allocated = True
        self._request_id = request_id
        self._resource = resource

    def abandon(self):
        self.allocated = False
        self._request_id = None
        self._resource = None

    def is_compatible(self, resource):
        return utils.is_compatible(self.msg, resource)
