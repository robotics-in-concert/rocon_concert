#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
# Used for running services that are defined by a multi-master style
# roslaunch - aka a link graph. In these services, the entities are all
# fixed and locked in (as well as their connections) from the start of the
# service to its termination.
#
##############################################################################
# Imports
##############################################################################

import copy
import rospy

import rocon_std_msgs.msg as rocon_std_msgs
import concert_msgs.msg as concert_msgs
import concert_service_utilities
import scheduler_msgs.msg as scheduler_msgs
import std_msgs.msg as std_msgs
import concert_schedulers
import yaml
import rocon_uri
import rocon_python_comms

##############################################################################
# Classes
##############################################################################


class StaticLinkGraphHandler(object):
    __slots__ = [
        '_name',
        '_description',
        '_priority',
        '_uuid',
        '_linkgraph',
        '_requester',
        'spin',
        '_subscribers',
        '_disabled'
    ]

    def __init__(self, name, description, priority, key, linkgraph):
        '''
          @param name
          @type str

          @param description
          @type string

          @param key
          @type uuid.UUID

          @param linkgraph
          @type concert_msgs.LinkGraph
        '''
        self._name = name
        self._description = description
        self._priority = priority
        self._uuid = key
        self._linkgraph = linkgraph
        self._disabled = False
        self._setup_resource_pool_requester()
        self._setup_ros_subscribers()

    def spin(self):
        while not rospy.is_shutdown() and not self._disabled:
            rospy.rostime.wallsleep(0.5)  # same period as rospy.client's spin

    def _setup_ros_subscribers(self):
        self._subscribers = {}
        # put the shutdown topic in the root of the service space
        self._subscribers['shutdown'] = rospy.Subscriber('shutdown', std_msgs.Empty, self._ros_subscriber_shutdown)

    def _ros_subscriber_shutdown(self, unused_msg):
        '''
          Typically called by the service manager when it wants to disable this service.
          Note: if the node is shutting down we don't want to run this code as it runs into all sorts
          of unreliable ros pubsub communication. Instead, we deallocate (stop apps and uninvite) via
          a shutdown hook in the conductor.

          @param unused_msg : it's just a signal to trigger this callback
          @type std_msgs.Empty
        '''
        rospy.loginfo("Service : disabling [%s]" % self._name)
        self._requester.cancel_all_requests()
        self._disabled = True

    def _setup_resource_pool_requester(self):
        '''
          Setup the resource groups, then feed it into and Initialise the
          resource pool requester, It looks everything from thereon.
        '''
        resource_groups = []
        for node in self._linkgraph.nodes:
            resources = []
            resource = _node_to_resource(node, self._linkgraph)
            for unused_i in range(node.max):
                resources.append(copy.deepcopy(resource))
            resource_groups.append(concert_schedulers.ResourcePoolGroup(node.min, resources))
        try:
            scheduler_requests_topic_name = concert_service_utilities.find_scheduler_requests_topic()
            #rospy.loginfo("Service : found scheduler [%s][%s]" % (topic_name))
        except rocon_python_comms.NotFoundException as e:
            rospy.logerr("Service : %s [%s]" % (str(e), self._name))
            return  # raise an exception here?
        self._requester = concert_schedulers.ResourcePoolRequester(
                                            resource_groups,
                                            feedback=self._requester_feedback,
                                            high_priority=self._priority,
                                            uuid=self._uuid,
                                            topic=scheduler_requests_topic_name
                                            )

    def _requester_feedback(self, request_set):
        '''
          Callback used to act on feedback coming from the scheduler request handler.

          @param request_set : a snapshot of the state of all requests from this requester
          @type concert_scheduler_requests.transition.RequestSet
        '''
        pass

##############################################################################
# Methods
##############################################################################


def load_linkgraph_from_yaml(yaml):
    """
        Loading a linkgraph from yaml and returns its name, and linkgraph

        :param str yaml: the link graph as a string loaded from yaml

        @return name - name of linkgraph
        @rtype str
        @return linkgraph
        @rtype concert_msgs.msg.LinkGraph
    """
    lg = concert_msgs.LinkGraph()
    name = yaml['name']
    for node in yaml['nodes']:
        node['min'] = node['min'] if 'min' in node else 1
        node['max'] = node['max'] if 'max' in node else 1
        node['force_name_matching'] = node['force_name_matching'] if 'force_name_matching' in node else False
        node['parameters'] = node['parameters'] if 'parameters' in node else {}
        lg.nodes.append(concert_msgs.LinkNode(node['id'], node['uri'], node['min'], node['max'], node['force_name_matching'],node['parameters']))
    for topic in yaml['topics']:
        lg.topics.append(concert_msgs.LinkConnection(topic['id'], topic['type']))
    if 'service' in yaml:
        for service in yaml['services']:
            lg.services.append(concert_msgs.LinkConnection(service['id'], service['type']))
    for action in yaml['actions']:
        lg.actions.append(concert_msgs.LinkConnection(action['id'], action['type']))
    for edge in yaml['edges']:
        lg.edges.append(concert_msgs.LinkEdge(edge['start'], edge['finish'], edge['remap_from'], edge['remap_to']))

    return name, lg


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
    with open(filename) as f:
        impl = yaml.load(f)
        name, lg = load_linkgraph_from_yaml(impl)

    return name, lg


def _node_to_resource(node, linkgraph):
    '''
      Convert linkgraph information for a particular node to a scheduler_msgs.Resource type.

      @param node : a node from the linkgraph
      @type concert_msgs.LinkNode

      @param linkgraph : the entire linkgraph (used to lookup the node's edges)
      @type concert_msgs.LinkGraph

      @return resource
      @rtype scheduler_msgs.Resource
    '''
    resource = scheduler_msgs.Resource()
    resource.rapp = rocon_uri.parse(node.resource).rapp
    resource.uri = node.resource
    resource.remappings = [rocon_std_msgs.Remapping(e.remap_from, e.remap_to) for e in linkgraph.edges if e.start == node.id or e.finish == node.id]
    resource.parameters = [rocon_std_msgs.KeyValue(key,str(val)) for key, val in node.parameters.items()]
    return resource
