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
from concert_service_manager import ServicePool
from concert_service_manager import InvalidSolutionConfigurationException

import rocon_console.console as console

##############################################################################
# Tests
##############################################################################

def test_valid_configuration():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Valid Solution Configuration" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
    service_pool = ServicePool('concert_service_manager/valid.services')
    print("%s" % service_pool)
    assert len(service_pool) == 3

 
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Invalid Solution Configurations" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
    for resource in ['concert_service_manager/duplicate_name.services', 'concert_service_manager/duplicate_resource_name.services']:
        with assert_raises(InvalidSolutionConfigurationException):
            print(" - " + console.green + resource + console.reset)
            service_pool = ServicePool(resource)
