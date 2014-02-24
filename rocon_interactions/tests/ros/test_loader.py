#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# RosTest
##############################################################################

""" Test loading of interactions to the interactions manager. """

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import unittest
import rostest
import rosunit
import rospy

##############################################################################
# Imports
##############################################################################

class TestLoader(unittest.TestCase):

    def setUp(self):
        rospy.init_node("loader")

    def test_loader(self):
        """ Loading... """
        rospy.logwarn("Dude")

    def tearDown(self):
        pass

if __name__ == '__main__':
    rostest.rosrun('rocon_interactions', 'loader', TestLoader)
