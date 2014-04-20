#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
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

      @param resource
      @type scheduler_msgs.Resource

      @param concert_client
      @type concert_msgs.ConcertClient
    '''
    if not rocon_uri.is_compatible(resource.uri, concert_client.platform_info.uri):
        return False
    for client_rapp in concert_client.rapps:
        if resource.rapp == client_rapp.name:
            return True
    return False
