#!/usr/bin/env python
import rospy
import yaml
import concert_service_roslaunch

from concert_msgs.srv import *
from concert_msgs.msg import *

def load_linkgraph_from_file(filename):
    lg = LinkGraph() 
    name = "None"

    with open(filename) as f:
        impl = yaml.load(f)
        name = impl['name'] 
        
        for node in impl['nodes']:
            node['min'] = node['min'] if 'min' in node else 1
            node['max'] = node['max'] if 'max' in node else 1
            node['force_name_matching'] = node['force_name_matching'] if 'force_name_matching' in node else False

            lg.nodes.append(LinkNode(node['id'], node['tuple'], node['min'], node['max'], node['force_name_matching']))
        for topic in impl['topics']:
            lg.topics.append(LinkConnection(topic['id'], topic['type']))

        if 'service' in impl:
            for service in impl['services']:
                lg.services.append(LinkConnection(service['id'], service['type']))
        for action in impl['actions']:
            lg.actions.append(LinkConnection(action['id'], action['type']))
        for edge in impl['edges']:
            lg.edges.append(LinkEdge(edge['start'], edge['finish'], edge['remap_from'], edge['remap_to']))

    return name, lg

if __name__ == '__main__':
    rospy.init_node('static_link_graph_service', anonymous=True)

    filename = rospy.get_param('~filename')

    impl_name, impl = load_linkgraph_from_file(filename)
    name = rospy.get_param("name",impl_name)

    sgsh =  concert_service_roslaunch.StaticLinkGraphHandler(name, impl)
    sgsh.spin()
