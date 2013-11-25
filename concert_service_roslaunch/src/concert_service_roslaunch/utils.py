#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import yaml

from concert_msgs.msg import *

##############################################################################
# Methods
##############################################################################


def load_linkgraph_from_file(filename):
    """
        Loading a linkgraph from file and returns its name, and linkgraph

        @param filename - yaml file
        @type str

        @return name - name of linkgraph
        @rtype str
        @return linkgraph
        @rtype concert_msgs.msg.LinkGraph
    """

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
