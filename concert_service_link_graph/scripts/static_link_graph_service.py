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
import concert_service_link_graph
import concert_service_utilities

from concert_msgs.srv import *
from concert_msgs.msg import *

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('static_link_graph_service', anonymous=True)

    # this is a uuid.UUID key
    (name, description, uuid) = concert_service_utilities.get_service_info()
    filename = rospy.get_param('~filename')

    impl_name, impl = concert_service_link_graph.load_linkgraph_from_file(filename)

    if not name:
        name = impl_name

    static_link_graph_service =  concert_service_link_graph.StaticLinkGraphHandler(name, description, uuid, impl)
    static_link_graph_service.spin()
