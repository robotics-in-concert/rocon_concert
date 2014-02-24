#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

#import rocon_std_msgs.msg as rocon_std_msgs
#import rocon_uri

##############################################################################
# Methods Used on concert_msgs.RemoconApp classes
##############################################################################


# def is_wildcard(element):
#     '''
#       Checks if an entry is a wildcard (i.e. '', or '*')
#     '''
#     return True if element == '' or element == '*' else False
#
#
# def is_smart_device(element):
#     return (element == rocon_std_msgs.PlatformTuple.PLATFORM_PHONE or
#         element == rocon_std_msgs.PlatformTuple.PLATFORM_TABLET or
#         element == rocon_std_msgs.PlatformTuple.PLATFORM_SMART_DEVICE)


# def is_runnable(remocon_app_uri, remocon_uri):
#     '''
#       Checks the given remocon uri and makes sure this app is compatible.
#       Used to filter apps from role-app lists provided to remocons.
#
#       @param remocon_app : check if this app is runnable.
#       @type concert_msgs.RemoconApp
#       @param remocon_uri : the rocon uri representing the remocon
#       @type str
#
#       @return true if compatible, false otherwise
#       @rtype Bool
#     '''
#     if remocon_uri is None:
#         return True
#     else:
#         return rocon_uri.is_compatible(app_uri, remocon_uri)
#     if (not is_wildcard(remocon_platform_info.tuple.os) and
#         not is_wildcard(remocon_app.platform_info.tuple.os) and
#         remocon_platform_info.tuple.os != remocon_app.platform_info.tuple.os):
#         return False
#     # Not worrying about version check yet (should need it for android soon)
#
#     if (not is_wildcard(remocon_platform_info.tuple.platform) and
#         not is_wildcard(remocon_app.platform_info.tuple.platform)):
#         if is_smart_device(remocon_platform_info.tuple.platform) and not is_smart_device(remocon_app.platform_info.tuple.platform):
#             return False
#         elif not is_smart_device(remocon_platform_info.tuple.platform) and is_smart_device(remocon_app.platform_info.tuple.platform):
#             return False
#         elif is_smart_device(remocon_platform_info.tuple.platform) and is_smart_device(remocon_app.platform_info.tuple.platform):
#             pass
#         elif remocon_platform_info.tuple.platform != remocon_app.platform_info.tuple.platform:
#             return False
    # Not worrying about system check yet
    # Not worrying about name check yet
#     return True


def matches(remocon_app_a, remocon_app_b):
    '''
      Checks if a matches b. Note that the internals of each can potentially
      be variable, specifically the platform info information.

      @todo update the check here as we get more elaborate.

      @param remocon_app_a : first app to check
      @type concert_msgs.RemoconApp

      @param remocon_app_b : second app to check
      @type concert_msgs.RemoconApp
    '''
    if remocon_app_a.name == remocon_app_b.name:
        return True
    return False


def is_app_in_app_list(remocon_app, remocon_app_list):
    '''
      Checks for a matching listing of the app in the app list.

      @param remocon_app
      @type concert_msgs.RemoconApp
      @param remocon_app_list
      @type concert_msgs.RemoconApp[]

      @return matching app or None if not found.
      @rtype concert_msgs.RemoconApp
    '''
    for app in remocon_app_list:
        if matches(remocon_app, app):
            return app
    return None
