#!/usr/bin/env python

import rospy
import copy
import concert_msgs.msg as concert_msg


def resolve(service_nodes, client_list):
    """
        @param service_nodes: list of service nodes.
        @type concert_msg.LinkNode[]

        @param client_list: list of client.
        @type concert_msg.ConcertClient[]

        @return result of pairing. message. list of valid pair (service node, client node)
        @rtype concert_msg.ErrorCodes.CONSTANT, string, [(LinkNode,ConcertClient)]
    """
#    rospy.loginfo("=== Clients ===")
#    print_dict(c_node)
#    rospy.loginfo("=== Service ===")
#    print_array(d_node)

    snodes = _expand_min_nodes(service_nodes)

    pairs = []
    result, message = _get_app_client_pair(pairs, snodes, client_list)

#rospy.loginfo("===== Result =====")
#rospy.loginfo(str(result))
#rospy.loginfo(str(message))
#print_pairs(pairs)
#rospy.loginfo("==================")
    return result, message, pairs


def _expand_min_nodes(nodes):
    """
        A multiple of same node may need. service description includes min-max value. Using min value it creates multiple instances.

        @param nodes list of service nodes
        @type concert_msgs.msg.ConcertService[]

        @return min expanded list of service nodes
        @rtype concert_msgs.msg.ConcertService[]
    """
    expanded_nodes = copy.deepcopy(nodes)

    for n in nodes:
        for unused_i in range((n.min - 1)):
            expanded_nodes.append(copy.deepcopy(n))

    return expanded_nodes


def _get_app_client_pair(pair, snodes, clients):
    """
        Simple backtracking to create pairs of service node and client

        @param pair: the list of currently created pair. It holds the full list of pairs after the full iteration
        @type list of (service_node, client)

        @param snodes: list of service nodes need to be paired
        @type concert_msgs.msg.LinkNode[]

        @param clients: list of remaining availalble clients
        @type concert_msgs.msg.ConcertClient[]

        @return result : constants in concert_msgs.ErrorCodeswhich indicates the status of pairing.
        @rtype int16

        @return message : comment
        @rtype string
    """
    result = concert_msg.ErrorCodes.SERVICE_UNEXPECTED_ERROR
    message = "No iteration yet"
    if len(snodes) == 0:
        return concert_msg.ErrorCodes.SUCCESS, "Successful Match Making"

    if len(clients) == 0:
        singles = [n.id for n in snodes]
        return concert_msg.ErrorCodes.SERVICE_INSUFFICIENT_CLIENTS, "No match for " + str(singles)

    snodes_copy = copy.deepcopy(snodes)
    clients_copy = copy.deepcopy(clients)

    for n in snodes:
        service_node_name = n.id
        _1, _2, _3, _4, service_app_name = n.tuple.split(".")
        for c in clients:
            if _is_valid_pair(n, c):

#                app = [ a for a in apps if a.name is app_name]
#                app_share = app[0].share
                p = (n, c)

                # Prepare for next depth
                pair.append(p)
                snodes_copy.remove(n)
                clients_copy.remove(c)

                # Go to next depth
                result, message = _get_app_client_pair(pair, snodes_copy, clients_copy)

                if result is concert_msg.ErrorCodes.SUCCESS:
                    return result, message

                # Return back to current depth
                pair.remove(p)
                snodes_copy.append(n)
                clients_copy.append(c)

    return result, message


def _is_valid_pair(n, c):
    """
        Compatibility check. Check if client node 'c' can serve as service node 'n'
        @param n : service node
        @type concert_msgs.msg.LinkNode

        @param c : client node
        @type concert_msgs.msg.ConcertClient

        @return True if client node 'c' can serve as service node 'n'
        @rtype bool
    """

    # 0 : os, 1: version, 2: system, 3: platform
    client_tuple = node_tuple = [0, 1, 2, 3]  # Man this is ugly. Impossible even to introspect n, c with prints to see wtf they are

    node_tuple[0], node_tuple[1], node_tuple[2], node_tuple[3], service_app_name = n.tuple.split(".")

    client_tuple[0] = c.os
    client_tuple[1] = c.version
    client_tuple[2] = c.system
    client_tuple[3] = c.platform

    client_apps = [a.name for a in c.apps]

    for i in range(3):
        if not (node_tuple[i] == client_tuple[i] or node_tuple[i] == '*'):
            return False

    if not (service_app_name in client_apps):
        return False

    return True


def print_dict(node):
    for n in node:
        rospy.loginfo(str(n) + " : " + str(node[n]))


def print_array(ary):
    for a in ary:
        rospy.loginfo(str(a))


def print_pairs(pairs):
    for p in pairs:
        n, c = p
        node_name = n.id
        rospy.loginfo(str(n.id) + " - " + str(c.name))
