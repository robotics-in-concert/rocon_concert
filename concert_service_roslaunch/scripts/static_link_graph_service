#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import yaml
import concert_service_roslaunch

from concert_msgs.srv import *
from concert_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('static_link_graph_service', anonymous=True)

    name = rospy.get_param("name")
    uuid = rospy.get_param("uuid")
    description = rospy.get_param("description")
    filename = rospy.get_param('~filename')

    impl_name, impl = concert_service_roslaunch.load_linkgraph_from_file(filename)

    if not name:
        name = impl_name

    sgsh =  concert_service_roslaunch.StaticLinkGraphHandler(name, uuid, description, impl)
    sgsh.spin()
