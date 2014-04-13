#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import sys
import unittest
import rospy
import rostest
import concert_service_utilities

##############################################################################
# Test
##############################################################################


class TestConcertInfo(unittest.TestCase):

    def __init__(self, *args):
        super(TestConcertInfo, self).__init__(*args)
        self._success = False

    def setUp(self):
        rospy.init_node('test_concert_info', anonymous=True)
        print("namespace: %s" % rospy.get_namespace())

    def test_concert_info(self):
        try:
            (name, unused_description, unused_priority, unused_uuid) = concert_service_utilities.get_service_info()
            if name == 'chatter':
                self._success = True
        except concert_service_utilities.ServiceInfoException:
            pass
        self.assert_(self._success)

    def tearDown(self):
        pass

if __name__ == '__main__':
    rostest.rosrun('concert_service_tutorials', 'test_concert_info', TestConcertInfo, sys.argv)
    #rospy.init_node('test_concert_info', anonymous=True)
    #rospy.spin()
