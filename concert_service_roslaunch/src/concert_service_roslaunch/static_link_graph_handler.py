#!/usr/bin/env python
#
# License: BSD
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

import rospy

import unique_id
import concert_msgs.msg as concert_msgs
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
        '_publishers',
    ]

    def __init__(self, name, description, uuid, linkgraph):
        '''
          @param name
          @type str

          @param description
          @type string

          @param uuid
          @type uuid.UUID

          @param linkgraph
          @type concert_msgs.LinkGraph
        '''
        self._name = name
        self._description = description
        self._uuid = uuid
        self._linkgraph = linkgraph
        self._publishers = {}
        self._setup_ros_api()

    def _setup_ros_api(self):
        self._publishers['request_resources'] = rospy.Publisher(concert_msgs.Strings.REQUEST_RESOURCES, concert_msgs.RequestResources, latch=True)

    def _request_resources(self, enable):
        rospy.loginfo("enable : " + str(enable))

        msg = concert_msgs.RequestResources()

        # Legacy style, delete once we go live.
        msg.service_name = self._name
        msg.linkgraph = self._linkgraph
        msg.enable = enable

        # Once this goes live, we can delete the legacy style above
        msg.id = unique_id.toMsg(unique_id.fromRandom())
        msg.group = self._name
        msg.resources = []
        msg.priority = 0
        for node in self._linkgraph.nodes:
            # node.tuple is of the form 'linux.*.ros.pc.rocon_apps/talker'
            resource = concert_msgs.Resource()
            (platform_part, unused_separator, resource.name) = node.tuple.rpartition('.')
            resource.platform_info = platform_part + "." + node.id
            resource.remappings = []
            # jihoon didn't do anything to implement for max values
            for unused_i in range(node.min):
                msg.resources.append(resource)
        self._publishers['request_resources'].publish(msg)

        # @TODO : Now we should regenerate the uuid, empty the resources, drop the
        # priority and traverse the nodes again. Add extras for any min-max ranges
        # (these are optionals)

    def spin(self):
        self._request_resources(True)
        rospy.spin()

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
