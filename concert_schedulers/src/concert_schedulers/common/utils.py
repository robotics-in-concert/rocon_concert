#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rocon_utilities.platform_tuples as platform_tuples

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
    if not platform_tuples.string_matches(concert_client.platform_info, resource.platform_info):
        return False
    for client_app in concert_client.apps:
        if resource.name == client_app.name:
            return True
    return False


def resource_uri_from_msg(msg):
    '''
      Converts a scheduler_msgs.Resource message to a resource uri string. e.g.

        linux.*.ros.Dude/rocon_apps/talker

      @param msg
      @type scheduler_msgs.Resource
    '''
    return msg.platform_info + "/" + msg.name
