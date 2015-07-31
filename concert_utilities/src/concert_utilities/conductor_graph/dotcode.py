#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rocon_gateway_utils
from rosgraph.impl.graph import Edge
import concert_msgs.msg as concert_msgs


##############################################################################
# Implementation
##############################################################################


class ConductorGraphDotcodeGenerator:

    def __init__(self):
        pass

    def _add_edge(self, edge, dotcode_factory, dotgraph):
#         if is_topic:
#             dotcode_factory.add_edge_to_graph(dotgraph, edge.start, edge.end, label=edge.label, url='topic:%s' % edge.label)
#         else:
        dotcode_factory.add_edge_to_graph(dotgraph, edge.start, edge.end, label=edge.label)

    def _add_conductor_node(self, dotcode_factory, dotgraph):
        '''
        The conductor is a special case node, we treat this especially here.
        '''
        dotcode_factory.add_node_to_graph(dotgraph,
                                          nodename="conductor",
                                          #nodename=rocon_gateway_utils.gateway_basename(node),
                                          #nodelabel=rocon_gateway_utils.gateway_basename(node),
                                          shape='ellipse',
                                          url="conductor",
                                          #url=node
                                          color="blue"
                                          )

    def _add_node(self, node, dotcode_factory, dotgraph):
        '''
        A node here is a concert client. We basically add nodes and classify according
        to their state in the dotgraph.

        :param node .concert_client.ConcertClient: the concert client to show on the dotgraph
        '''
        # colour strings defined as per http://qt-project.org/doc/qt-4.8/qcolor.html#setNamedColor
        # and http://www.w3.org/TR/SVG/types.html#ColorKeywords
        if node.state == concert_msgs.ConcertClientState.PENDING:
            node_colour = "magenta"
        elif node.state == concert_msgs.ConcertClientState.JOINING:
            node_colour = "magenta"
        elif node.state == concert_msgs.ConcertClientState.UNINVITED:
            node_colour = "midnightblue"
        elif node.state == concert_msgs.ConcertClientState.AVAILABLE:
            node_colour = "blue"
        elif node.state == concert_msgs.ConcertClientState.MISSING:
            node_colour = "powderblue"
        elif node.state == concert_msgs.ConcertClientState.BLOCKING:
            node_colour = "black"
        elif node.state == concert_msgs.ConcertClientState.BUSY:
            node_colour = "black"
        elif node.state == concert_msgs.ConcertClientState.GONE:
            node_colour = "black"
        elif node.state == concert_msgs.ConcertClientState.BAD:
            node_colour = "red"
        dotcode_factory.add_node_to_graph(dotgraph,
                                          nodename=node.concert_alias,
                                          #nodename=rocon_gateway_utils.gateway_basename(node),
                                          #nodelabel=rocon_gateway_utils.gateway_basename(node),
                                          shape='ellipse',
                                          url=rocon_gateway_utils.gateway_basename(node.gateway_name),
                                          #url=node,
                                          color=node_colour
                                          )

    def get_nodes_and_edges(self, conductor_graph_instance):
        """
        Get all the nodes and edges corresponding to our conductor's graph of concert clients.

        :returns: all the nodes and edges
        :rtype: (concert_client.ConcertClient[], rosgraph.impl.graph.Edge[])
        """
        nodes = conductor_graph_instance.concert_clients.values()
        edges = []
        important_states = [
                        concert_msgs.ConcertClientState.MISSING,
                        concert_msgs.ConcertClientState.AVAILABLE
                       ]
        for node in nodes:
            if node.msg.state in important_states:  # and node.msg.conn_stats.gateway_available:
                edges.append(Edge("conductor", node.concert_alias, node.link_type))
        return (nodes, edges)

    def generate_dotcode(self,
                         conductor_graph_instance,
                         dotcode_factory,
                         clusters=False,
                         uninvited_filter=False,  # not yet used
                         orientation='LR',
                         rank='same',
                         ranksep=0.2,
                         rankdir='TB',
                         simplify=True,
                         ):
        """
        :param conductor_graph_instance ConductorGraphInfo: all the information behind the graph
        :param dotcode_factory: abstract factory manipulating dot language objects
        :param clusters bool: whether to show by ip clusters or not
        :param uninvited_filter: disable viewing of uninvited clients
        :param orientation: rankdir value (see ORIENTATIONS dict)
        :param rank str: one of None, same, min, max, source, sink
        :param ranksep float: vertical distance between layers
        :param rankdir str: direction of layout (TB top > bottom, LR left > right)
        :param simplify bool: remove double edges
        :returns: dotcode generated from graph singleton
        :rtype: str
        """
        (nodes, edges) = self.get_nodes_and_edges(conductor_graph_instance)
        dotgraph = dotcode_factory.get_graph(rank=rank,
                                             ranksep=ranksep,
                                             simplify=simplify,
                                             rankdir=orientation)

        ip_clusters = {}
        if conductor_graph_instance.is_conductor:
            self._add_conductor_node(dotcode_factory=dotcode_factory, dotgraph=dotgraph)
        if nodes is not None:
            for node in nodes:
                if clusters:
                    if node.ip not in ip_clusters.keys():
                        ip_clusters[node.ip] = dotcode_factory.add_subgraph_to_graph(dotgraph, node.ip, rank=rank, rankdir=orientation, simplify=simplify)
                    self._add_node(node, dotcode_factory=dotcode_factory, dotgraph=ip_clusters[node.ip])
                else:
                    self._add_node(node, dotcode_factory=dotcode_factory, dotgraph=dotgraph)
        for e in edges:
            self._add_edge(e, dotcode_factory, dotgraph=dotgraph)

        dotcode = dotcode_factory.create_dot(dotgraph)
        return dotcode

class ConductorStateDotcodeGenerator:
    '''
        Unlike Conductor graph focuses on visualising status of clients, ConductorState focuses on states and transitions.
    '''

    def __init__(self, dotcode_factory):
        self._dotcode_factory = dotcode_factory
        pass

    def generate_dotcode(self, state_transition_table, **kargs):
        dotgraph = self.generate_dotgraph(state_transition_table, **kargs)
        dotcode = self._dotcode_factory.create_dot(dotgraph)
        return dotcode

    def generate_dotgraph(self, state_transition_table, rank='same',ranksep=0.5, rankdir='TB', simplify=True):
        nodes, edges = self.get_nodes_and_edges(state_transition_table)
        dotgraph = self._dotcode_factory.get_graph(rank=rank, ranksep=ranksep, simplify=simplify, rankdir=rankdir)
        self._add_not_in_network_node_and_edge(dotgraph)

        for n in nodes:
            self._add_node(dotgraph, n)

        for e in edges:
            self._add_edge(dotgraph, e)
        return dotgraph

    def get_nodes_and_edges(self, state_transition_table):
        nodes = []
        edges = []

        for transition, transition_func in state_transition_table.items():
            fr, to = transition
            nodes.append(fr)
            nodes.append(to)

            label = '' if transition_func.__name__ == 'Dummy' else transition_func.__name__
        
            edges.append(Edge(fr, to, label))

        # make it as unique string list
        nodes = list(set(nodes))

        return nodes, edges
    
    def _add_not_in_network_node_and_edge(self, dotgraph):
        start_node_name = "not in network"
        self._dotcode_factory.add_node_to_graph(dotgraph, start_node_name, shape='ellipse', color="blue")
        self._dotcode_factory.add_edge_to_graph(dotgraph, start_node_name, concert_msgs.ConcertClientState.PENDING, penwidth=0.3)
        self._dotcode_factory.add_edge_to_graph(dotgraph, concert_msgs.ConcertClientState.GONE, start_node_name, penwidth=0.3)


    def _add_node(self, dotgraph, node):
        self._dotcode_factory.add_node_to_graph(dotgraph, nodename = node, shape='ellipse')

    def _add_edge(self, dotgraph, edge):
        self._dotcode_factory.add_edge_to_graph(dotgraph, edge.start, edge.end, label=edge.label, penwidth=0.3)
