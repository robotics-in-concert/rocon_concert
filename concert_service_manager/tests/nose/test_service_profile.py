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

import hashlib

import rospkg
import rocon_python_utils
import rocon_console.console as console

from concert_service_manager import ServiceProfile
    
##############################################################################
# Tests
##############################################################################

def test_valid_profile():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Valid Service Profiles" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
    resource_name = 'concert_service_manager/valid.service'
    overrides = {}
    location_cache = {}
    service_profile = ServiceProfile(hashlib.sha224('fake'), resource_name, overrides, location_cache)
    print(console.bold + "Service Profile:\n" + console.reset + str(service_profile))
    assert service_profile.name == "chatter"
    overrides = { 'name': 'babbler'}
    service_profile = ServiceProfile(hashlib.sha224('fake'), resource_name, overrides, location_cache)
    assert service_profile.name == "babbler"
    print(console.bold + "Service Profile:\n" + console.reset + str(service_profile))
