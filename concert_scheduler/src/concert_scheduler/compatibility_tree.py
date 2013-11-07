#!/usr/bin/env python

import rospy
import copy
import concert_msgs.msg as concert_msg


def resolve(nodes, client_list):
    """
        @param
            nodes: list of service nodes.
            client_list: list of client
        @return
            result: Whether it is successful or not.
            message: Comment on result
            pair: list of valid pair (service node, client node, client gateway name)
    """
    d_node = _create_service_array(nodes)

    # create app indexed dict
    c_node = _create_client_dict(client_list)

#    rospy.loginfo("=== Clients ===")
#    print_dict(c_node)
#    rospy.loginfo("=== Service ===")
#    print_array(d_node)

    pair = []
    result, message = _get_app_client_pair(pair, d_node, c_node)

#    rospy.loginfo("== Result")
#    print_array(pair)
#    rospy.loginfo("================")
    return result, message, pair


def _create_service_array(nodes):
    """
        @param
            nodes: list of concert_msg.LinkNode
        @return
            node: list of tuples of (os, version, system, platform, app, node id)
    """
    node = []
    for n in nodes:
        os, version, system, platform, app = n.tuple.split(".")
        for unused_i in range(n.min):
            node.append((os, version, system, platform, app, n.id))
    return node


def _get_app_client_pair(pair, n_node, c_node):
    """
        Simple backtracking to create pairs of service node and client
        @param
            pair: the list of currently created pair. It holds the full list of pairs after the full iteration
            n_node: the list of service node has not assigned yet as pair
            c_node: the list of client node has not assigned as pair
        @return
            result: constants in concert_msgs.ConcertService which indicates the status of pairing
            message: comment string
    """
    result = concert_msg.ConcertService.UNEXPECTED_ERROR
    message = "No iteration yet"
    if len(n_node) == 0:
        return concert_msg.ConcertService.READY, "Successful Match Making"

    if len(c_node) == 0:
        singles = [name for _n0, _n1, _n2, _n3, name in n_node]
        return concert_msg.ConcertService.INSUFFICIENT_CLIENTS, "No match for " + str(singles)

    nn_node = copy.deepcopy(n_node)
    cc_node = copy.deepcopy(c_node)

    for n in n_node:
        for c in c_node:
            if _is_valid_pair(n, c_node[c]):
                _1, _2, _3, _4, _5, gatewayname = c_node[c]
                p = (n, c, gatewayname)

                # Prepare for next depth
                pair.append(p)
                nn_node.remove(n)
                del cc_node[c]

                # Go to next depth
                result, message = _get_app_client_pair(pair, nn_node, cc_node)

                if result is concert_msg.ConcertService.READY:
                    return result, message

                # Return back to current depth
                pair.remove(p)
                nn_node.append(n)
                cc_node[c] = c_node[c]

    return result, message


def _is_valid_pair(n, c):
    """
        Compatibility check. Check if client node 'c' can serve as service node 'n'
        @param
            n: service node (os.version.system.platform.app_name.node_name)
            c: client node  (os.version.system.platform.app_list.gateway_name)
    """

    # 0 : os, 1: version, 2: system, 3: platform
    client = node = [0, 1, 2, 3]  # Man this is ugly. Impossible even to introspect n, c with prints to see wtf they are
    node[0], node[1], node[2], node[3], app, name = n
    client[0], client[1], client[2], client[3], apps, gatewayname = c

    for i in range(3):
        if not (node[i] == client[i] or node[i] == '*'):
            return False

    if not (app in apps):
        return False

    return True


def print_dict(c_node):
    for c in c_node:
        rospy.loginfo(str(c) + " : " + str(c_node[c]))


def print_array(pair):
    for p in pair:
        rospy.loginfo(str(p))


def _create_client_dict(client_list):
    c_node = {}
    for c in client_list:

        apps = [a.name for a in c.apps]
        c_node[c.name] = (c.os, c.version, c.system, c.platform, apps, c.gateway_name)

    return c_node
