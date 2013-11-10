#! /usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import yaml
from concert_msgs.srv import *
from concert_msgs.msg import *


def load_service_from_file(filename):
    rs = ConcertService()

    with open(filename) as f:
        yaml_data = yaml.load(f)
        rs.name = yaml_data['name']
        rs.description = yaml_data['description']
        rs.author = yaml_data['author']
        rs.priority = yaml_data['priority']
        rs.launcher = yaml_data['launcher']
        rs.interactions = yaml_data['interactions'] if yaml_data.has_key('interactions') else '' 
#rs.linkgraph = linkgraph_to_msg(yaml_data['linkgraph'])

        t = yaml_data.get('launcher_type', 'custom')

        if t == 'roslaunch':
            rs.launcher_type = ConcertService.TYPE_ROSLAUNCH
        else:
            rs.launcher_type = ConcertService.TYPE_CUSTOM
    return rs

rospy.init_node('add_srv')

filename = rospy.get_param('~filename')

rs = load_service_from_file(filename)

rospy.wait_for_service('services/add')
srv = rospy.ServiceProxy('services/add', AddConcertService)

r = srv(rs)
