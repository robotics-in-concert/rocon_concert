#! /usr/bin/env python

import rospy
import yaml
from concert_msgs.srv import *
from concert_msgs.msg import *

def load_service_from_file(filename):
    rs = ConcertService()

    with open(filename) as f:
        yaml_data = yaml.load(f)
        rs.name         = yaml_data['name']
        rs.description  = yaml_data['description']
        rs.author       = yaml_data['author']

    return rs



rospy.init_node('add_srv')

filename = rospy.get_param('~filename')

rs = load_service_from_file(filename)

rospy.wait_for_service('service/add')
srv = rospy.ServiceProxy('service/add',AddConcertService)

r = srv(rs)
rospy.loginfo(str(r))
