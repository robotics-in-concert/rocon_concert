#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

from nose.tools import assert_raises, assert_false

import rospkg
import rocon_python_utils
import rocon_console.console as console

from concert_service_manager import load_service_profile

##############################################################################
# Utilities
##############################################################################


def pretty_print(service_profile):
    print(" - " + console.green + "%s" % service_profile.resource_name + console.reset)
    print("     " + console.cyan + "name: " + console.yellow + "%s" % service_profile.name + console.reset)
    
##############################################################################
# Tests
##############################################################################

# def test_valid_profile():
#     print(console.bold + "\n****************************************************************************************" + console.reset)
#     print(console.bold + "* Valid Service Profiles" + console.reset)
#     print(console.bold + "****************************************************************************************" + console.reset)
#     print("")
#     resource_name = 'concert_service_manager/valid.service'
#     filename = rocon_python_utils.ros.find_resource_from_string(resource_name)
#     overrides = {}
#     service_profile = load_service_profile(resource_name, overrides, filename)
#     assert service_profile.name == "chatter"
#     pretty_print(service_profile)
#     overrides = { 'name': 'babbler'}
#     service_profile = load_service_profile(resource_name, overrides, filename)
#     assert service_profile.name == "babbler"
#     pretty_print(service_profile)
