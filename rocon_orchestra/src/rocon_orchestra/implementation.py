#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_orchestration/rocon_orchestra/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import re
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs
import pydot

##############################################################################
# Classes
##############################################################################


class Implementation:
    '''
      If a solution implementation is being loaded, this stores the data.
    '''
    def __init__(self):
        # This will need some modification if we go to multiple solutions on file.
        self._name = rospy.get_param("~name", "Implementation 42")
        self.nodes = rospy.get_param("~nodes", [])
        self._topics = rospy.get_param("~topics", [])
        self._actions = rospy.get_param("~actions", [])
        self._edges = rospy.get_param("~edges", [])
        self._dot_graph = rospy.get_param("~dot_graph", "")
        self._implementation_publisher = rospy.Publisher("implementation", concert_msgs.Implementation, latch=True)
        self.publish()
        rospy.loginfo("Orchestration : initialised the implementation server.")

    def publish(self):
        self._implementation_publisher.publish(self.to_msg())

    def to_msg(self):
        '''
          Might be easier just serving up the whole implementation file and saving that
          in a string here.
        '''
        implementation = concert_msgs.Implementation()
        implementation.name = self._name
        for node in self.nodes:
            implementation.link_graph.nodes.append(concert_msgs.LinkNode(node['id'], node['tuple']))
        for topic in self._topics:
            implementation.link_graph.topics.append(concert_msgs.LinkConnection(topic['id'], topic['type']))
        for action in self._actions:
            implementation.link_graph.actions.append(concert_msgs.LinkConnection(action['id'], action['type']))
        for edge in self._edges:
            implementation.link_graph.edges.append(concert_msgs.LinkEdge(edge['start'], edge['finish'], edge['remap_from'], edge['remap_to']))
        implementation.dot_graph = self._dot_graph
        return implementation

    def rebuild(self, node_client_pairs):
        '''
          If the node name and client name don't match, rebuild

          @param node_client_pairs : list of node id-client name pairs
        '''
        for pair in node_client_pairs:
            node_id = pair[0]
            client_name = pair[1]
            if node_id != client_name:
                for node in self.nodes:
                    if node['id'] == node_id:
                        node['id'] = client_name
                for topic in self._topics:
                    if re.search(node_id, topic['id']):
                        topic['id'] = topic['id'].replace(node_id, client_name)
                for action in self._actions:
                    if re.search(node_id, action['id']) is not None:
                        action['id'] = action['id'].replace(node_id, client_name)
                for edge in self._edges:
                    if edge['start'] == node_id:
                        edge['start'] = client_name
                    if edge['finish'] == node_id:
                        edge['finish'] = client_name
                    if re.search(node_id, edge['remap_from']):
                        edge['remap_from'] = edge['remap_from'].replace(node_id, client_name)
                    if re.search(node_id, edge['remap_to']):
                        edge['remap_to'] = edge['remap_to'].replace(node_id, client_name)

    def to_dot(self):
        graph = pydot.Dot(graph_type='graph')
#        for node in self.nodes:
#            n = pydot.Node(node['id'], style="filled", fillcolor="red")
#            graph.add_node(n)
#        self.nodes = rospy.get_param("~nodes", [])
#        self._topics = rospy.get_param("~topics", [])
#        self._actions = rospy.get_param("~actions", [])
#        self._edges = rospy.get_param("~edges", [])
#        self._dot_graph = rospy.get_param("~dot_graph", "")
#        self._implementation_publisher = rospy.Publisher("implementation", concert_msgs.Implementation, latch=True)
#        self.publish()
