#!/usr/bin/env python
import rospy
import yaml
from concert_msgs.srv import *
from concert_msgs.msg import *

def load_linkgraph_from_file(filename):
    lg = LinkGraph() 

    with open(filename) as f:
        imple = yaml.load(f)
        
        for node in impl['nodes']:
            node['min'] = node['min'] if 'min' in node else 1
            node['max'] = node['max'] if 'max' in node else 1

            lg.nodes.append(LinkNode(node['id'], node['tuple'], node['min'], node['max'], node['force_name_matching']))
            for topic in impl['topics']:
                lg.topics.append(LinkConnection(topic['id'], topic['type']))
            for action in impl['actions']:
                lg.actions.append(LinkConnection(action['id'], action['type']))
            for edge in impl['edges']:
                lg.edges.append(LinkEdge(edge['start'], edge['finish'], edge['remap_from'], edge['remap_to']))

    return lg

if __name__ == '__main__':
    rospy.init_node('static_link_graph_service')

    filename = rospy.get_param('~filename')

    impl = load_from_file(filename)

    sgsh =  StaticGraphServiceHandler(impl)
    sgsh.spin()
