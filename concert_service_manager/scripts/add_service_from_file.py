#! /usr/bin/env python

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
        rs.linkgraph = linkgraph_to_msg(yaml_data['linkgraph'])
        rs.priority = yaml_data['priority']

    return rs


def linkgraph_to_msg(impl):

    lg = LinkGraph()

    for node in impl['nodes']:
        node['min'] = node['min'] if 'min' in node else 1
        node['max'] = node['max'] if 'max' in node else 1

        lg.nodes.append(LinkNode(node['id'], node['tuple'], node['min'], node['max'], False))
    for topic in impl['topics']:
        lg.topics.append(LinkConnection(topic['id'], topic['type']))
    for action in impl['actions']:
        lg.actions.append(LinkConnection(action['id'], action['type']))
    for edge in impl['edges']:
        lg.edges.append(LinkEdge(edge['start'], edge['finish'], edge['remap_from'], edge['remap_to']))

    return lg

rospy.init_node('add_srv')

filename = rospy.get_param('~filename')

rs = load_service_from_file(filename)

rospy.wait_for_service('service/add')
srv = rospy.ServiceProxy('service/add', AddConcertService)

r = srv(rs)
#rospy.loginfo(str(r))
