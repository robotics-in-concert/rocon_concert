#!/usr/bin/env python

import rospy
import copy
import concert_msgs.msg as concert_msgs
import rocon_utilities


def resolve(resources, client_list):
    """
        @param resources: list of requested nodes.
        @type scheduler_msgs.Resource[]

        @param client_list: list of client.
        @type concert_msg.ConcertClient[]

        @return result of pairing. message. list of valid pair (service node, client node)
        @rtype concert_msg.ErrorCodes.CONSTANT, string, [(LinkNode,ConcertClient)]
    """
    pairs = []
    result, message = _get_app_client_pair(pairs, resources, client_list)
    return result, message, pairs


def _get_app_client_pair(pairs, resources, clients):
    """
        Simple backtracking to create pairs of service node and client

        @param pairs: the list of currently created pairs. It holds the full list of pairs after the full iteration
        @type list of (node, client)

        @param nodes: list of service nodes need to be paired
        @type scheduler_msgs.Resource

        @param clients: list of remaining available clients
        @type concert_msgs.msg.ConcertClient[]

        @return result : constants in concert_msgs.ErrorCodeswhich indicates the status of pairing.
        @rtype int16

        @return message : comment
        @rtype string
    """
    result = None
    message = "No iteration yet"
    if len(resources) == 0:
        return concert_msgs.ErrorCodes.SUCCESS, "Successful Match Making"

    if len(clients) == 0:
        #singles = [_get_resource_platform_name(resource) for resource in resources]
        return concert_msgs.ErrorCodes.SERVICE_INSUFFICIENT_CLIENTS, "Insufficient clients for resources"

    nodes_copy = copy.deepcopy(resources)
    clients_copy = copy.deepcopy(clients)

    for resource in resources:
        for c in clients:
            if _is_valid_pair(resource, c):

                p = (resource, c)

                # Prepare for next depth
                pairs.append(p)
                nodes_copy.remove(resource)
                clients_copy.remove(c)

                # Go to next depth
                result, message = _get_app_client_pair(pairs, nodes_copy, clients_copy)

                if result is not None:  # error
                    return result, message

                if result is concert_msgs.ErrorCodes.SUCCESS:
                    return result, message

                # Return back to current depth
                pairs.remove(p)
                nodes_copy.append(resource)
                clients_copy.append(c)

    return concert_msgs.ErrorCodes.SERVICE_UNEXPECTED_ERROR, message


def _is_valid_pair(resource, c):
    """
        Compatibility check. Check if client node 'c' can serve as service node 'n'
        @param resource : requested resource
        @type scheduler_msgs.Resource

        @param c : client node
        @type concert_msgs.msg.ConcertClient

        @return True if client node 'c' can serve as service node 'n'
        @rtype bool
    """
    # 0 : os, 1: version, 2: system, 3: platform
    client_tuple = node_tuple = [0, 1, 2, 3]  # Man this is ugly. Impossible even to introspect n, c with prints to see wtf they are

    service_app_name = resource.name

    # This is really quite fugly.
    client_platform_info = rocon_utilities.platform_info.to_msg(c.platform_info)
    node_tuple[0], node_tuple[1], node_tuple[2], node_tuple[3], unused_name = resource.platform_info.split(".")

    client_tuple[0] = client_platform_info.os
    client_tuple[1] = client_platform_info.version
    client_tuple[2] = client_platform_info.system
    client_tuple[3] = client_platform_info.platform

    client_apps = [a.name for a in c.apps]

    for i in range(3):
        if not (node_tuple[i] == client_tuple[i] or node_tuple[i] == '*'):
            return False

    if not (service_app_name in client_apps):
        return False

    return True


###############################################################################
# Utilities
###############################################################################

def _get_resource_platform_name(resource):
    '''
      @param resource : details about a requested resource
      @type scheduler_msgs.Resource

      @return the platform name buried at the end of the platform_info string tuple.
      @type str
    '''
    (unused_platform_part, unused_separator, name) = resource.platform_info.rpartition('.')
    return name


def print_pairs(pairs):
    for pair in pairs:
        resource, client = pair
        resource_name = _get_resource_platform_name(resource)
        rospy.loginfo(str(resource_name) + " - " + str(client.name))
