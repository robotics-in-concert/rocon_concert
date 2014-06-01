#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
"""
.. module:: common.utils

Simple utilities for the concert schedulers and requesters.
"""
##############################################################################
# Imports
##############################################################################

import rocon_uri

# local imports

##############################################################################
# Methods
##############################################################################


def is_compatible(concert_client, resource):
    '''
      Checks to see if a client is compatible for the implementation's node rule.

      :param scheduler_msgs.Resource resource:
      :param concert_msgs.ConcertClient: concert_client
      :returns: true if compatible, false otherwise
      :rtype: bool
    '''
    if not rocon_uri.is_compatible(resource.uri, concert_client.platform_info.uri):
        return False
    for client_rapp in concert_client.rapps:
        if resource.rapp == client_rapp.name:
            return True
    return False
