#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
# Used for runnings services that are defined by a multi-master style
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
import rocon_scheduler_requests
import scheduler_msgs.msg as scheduler_msgs
import concert_schedulers
import yaml


##############################################################################
# Classes
##############################################################################


class StaticLinkGraphHandler(object):
    __slots__ = [
        '_name',
        '_description',
        '_uuid',
        '_linkgraph',
        '_requester',
        '_param',
        'spin',
        '_subscribers',
        '_disabled'
    ]

    def __init__(self, name, description, key, linkgraph):
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
        self._uuid = key
        self._linkgraph = linkgraph
        self._param = setup_ros_parameters()
        self._disabled = False
        if (self._param['requester_type'] == 'resource_pool_requester'):
            self._setup_resource_pool_requester()
        else:
            self._setup_requester()

    def spin(self):
        while not rospy.is_shutdown() and not self._disabled:
            rospy.rostime.wallsleep(0.5)  # same period as rospy.client's spin

    def _setup_ros_subscribers(self):
        self._subscribers = {}
        self._subscribers['disable'] = rospy.Subscriber('~disable', concert_msgs.ConcertClients, self._ros_subscriber_disable)

    def _ros_subscriber_disable(self):
        rospy.loginfo("Service : disabling [%s]" % self._name)
        rospy.logwarn("Service : this is a stub for sending the requester a cancel_all_requests request [%s]" % self._name)
        #self._requester.cancel_all_requests()
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
        self._requester = concert_schedulers.ResourcePoolRequester(
                                            resource_groups,
                                            feedback=self._requester_feedback,
                                            uuid=self._uuid,
                                            topic=concert_msgs.Strings.SCHEDULER_REQUESTS
                                            )

    def _setup_requester(self):
        '''
          Initialise the requester, then feed it a sequence of requests - 1 for the
          essential resources (up to min) and 1 each for any resource thereafter.
        '''
        self._requester = rocon_scheduler_requests.Requester(feedback=self._requester_feedback,
                                            uuid=self._uuid,
                                            topic=concert_msgs.Strings.SCHEDULER_REQUESTS
                                            )
        resources = []
        for node in self._linkgraph.nodes:
            # node.tuple is of the form 'linux.*.ros.pc.rocon_apps/talker'
            resource = _node_to_resource(node, self._linkgraph)
            for unused_i in range(node.min):
                resources.append(resource)
        unused_request_uuid = self._requester.new_request(resources)

        # provide extra requests for over min, and under max
        # unfortunately no priority settable here yet so this doesn't
        # yet work well - they get in the way of each other.
        # in fact it works horribly, since dudette in chatter_concert will often get assigned
        # to the listener here.
        #for node in self._linkgraph.nodes:
        #    for unused_i in range(node.max - node.min):
        #        resource = _node_to_resource(node, self._linkgraph)
        #        # could use a way of setting the request priority here.
        #        unused_request_uuid = self._requester.new_request([resource])

    def _requester_feedback(self, request_set):
        '''
          Callback used to act on feedback coming from the scheduler request handler.

          @param request_set : a snapshot of the state of all requests from this requester
          @type rocon_scheduler_requests.transition.RequestSet
        '''
        pass

##############################################################################
# Methods
##############################################################################


def setup_ros_parameters():
    param = {}
    param['requester_type'] = rospy.get_param('~requester_type', 'demo')
    return param


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

    lg = concert_msgs.LinkGraph()

    with open(filename) as f:
        impl = yaml.load(f)
        name = impl['name']

        for node in impl['nodes']:
            node['min'] = node['min'] if 'min' in node else 1
            node['max'] = node['max'] if 'max' in node else 1
            node['force_name_matching'] = node['force_name_matching'] if 'force_name_matching' in node else False

            lg.nodes.append(concert_msgs.LinkNode(node['id'], node['tuple'], node['min'], node['max'], node['force_name_matching']))
        for topic in impl['topics']:
            lg.topics.append(concert_msgs.LinkConnection(topic['id'], topic['type']))

        if 'service' in impl:
            for service in impl['services']:
                lg.services.append(concert_msgs.LinkConnection(service['id'], service['type']))
        for action in impl['actions']:
            lg.actions.append(concert_msgs.LinkConnection(action['id'], action['type']))
        for edge in impl['edges']:
            lg.edges.append(concert_msgs.LinkEdge(edge['start'], edge['finish'], edge['remap_from'], edge['remap_to']))
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
    (platform_part, unused_separator, resource.name) = node.tuple.rpartition('.')
    resource.platform_info = platform_part + "." + node.id
    if node.force_name_matching:
        resource.platform_info = platform_part + "." + node.id
    else:
        resource.platform_info = platform_part + "." + rocon_std_msgs.PlatformInfo.NAME_ANY
    resource.remappings = [rocon_std_msgs.Remapping(e.remap_from, e.remap_to) for e in linkgraph.edges if e.start == node.id or e.finish == node.id]
    return resource
