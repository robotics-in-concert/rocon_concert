'''
Created on 22/11/2011

@author: snorri
'''
##############################################################################
# Imports
##############################################################################

import roslib
roslib.load_manifest('rocon_orchestra')
import rospy

import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs

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
        self._nodes = rospy.get_param("~nodes", [])
        self._topics = rospy.get_param("~topics", [])
        self._actions = rospy.get_param("~actions", [])
        self._edges = rospy.get_param("~edges", [])
        self._dot_graph = rospy.get_param("~dot_graph", "")
        self._implementation_server = rospy.Service('~implementation', concert_srvs.Implementation, self.serve_implementation_details)
        rospy.loginfo("Orchestration : initialised the implementation server.")

    def serve_implementation_details(self, req):
        '''
          Might be easier just serving up the whole implementation file and saving that
          in a string here.
        '''
        implementation = concert_srvs.ImplementationResponse()
        implementation.name = self._name
        for node in self._nodes:
            implementation.link_graph.nodes.append(concert_msgs.LinkNode(node['id'], node['tuple']))
        for topic in self._topics:
            implementation.link_graph.topics.append(concert_msgs.LinkConnection(topic['id'], topic['name'], topic['type']))
        for action in self._actions:
            implementation.link_graph.actions.append(concert_msgs.LinkConnection(action['id'], action['name'], action['type']))
        for edge in self._edges:
            implementation.link_graph.edges.append(concert_msgs.LinkEdge(edge['start'], edge['finish']))
        implementation.dot_graph = self._dot_graph
        return implementation
