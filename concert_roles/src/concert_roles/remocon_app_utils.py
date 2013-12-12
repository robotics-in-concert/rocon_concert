#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

##############################################################################
# Methods Used on concert_msgs.RemoconApp classes
##############################################################################


def is_runnable(remocon_app, remocon_platform_info):
    '''
      Checks the given remocon platform info and makes sure this app is compatible.
      Used to filter apps from role-app lists provided to remocons.

      @param remocon_app : check if this app is runnable.
      @type concert_msgs.RemoconApp
      @param remocon_platform_info : the remocon platform to run on.
      @type concert_msgs.PlatformInfo

      @return true if compatible, false otherwise
      @rtype Bool
    '''
    if remocon_platform_info is None:
        return True
    if remocon_platform_info.os == '' or remocon_platform_info.os == '*':
        return True
    if remocon_app.platform_info.os == '' or remocon_app.platform_info.os == '*':
        return True
    elif remocon_platform_info.os != remocon_app.platform_info.os:
        return False
    # Not worrying about version check yet (should need it for android soon)
    # Not worrying about platform check yet
    # Not worrying about system check yet
    # Not worrying about name check yet
    return True


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
