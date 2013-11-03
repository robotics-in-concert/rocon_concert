#!/usr/bin/env python

import rospy
import copy
import concert_msgs.msg as concert_msg

def resolve(dedicated_nodes,client_list): 

    d_node = create_service_array(dedicated_nodes)

    # create app indexed dict
    c_node = create_client_dict(client_list)

    rospy.loginfo("=== Clients ===")
    print_dict(c_node)
    rospy.loginfo("=== Service ===")
    print_array(d_node)

    pair = []
    result, message = get_app_client_pair(pair,d_node,c_node)

    rospy.loginfo("== Result")
    print_array(pair)
    rospy.loginfo("================")
    return result, message, pair

def create_service_array(nodes):
    node = []
    for n in nodes:
        platform, system, robot, app = n.tuple.split(".")
                                                                   
        for i in range(n.min):
            node.append((platform, system, robot, app, n.id))
    return node





def get_app_client_pair(pair,n_node,c_node):
    result = concert_msg.ConcertService.UNEXPECTED_ERROR 
    message = "No iteration yet"
    if len(n_node) == 0:
        return True, "Successful Match Making"

    if len(c_node) == 0:
        singles = [name for _n0, _n1, _n2, _n3, name in n_node]
        return False, "No match for " + str(singles)

    nn_node = copy.deepcopy(n_node)
    cc_node = copy.deepcopy(c_node)

    for n in n_node:
        for c in c_node:
            if is_valid_pair(n,c_node[c]):
                _1,_2,_3, _4, gatewayname = c_node[c]
                p = (n, c, gatewayname)

                # Prepare for next depth
                pair.append(p)
                nn_node.remove(n)
                del cc_node[c]

                # Go to next depth
                result, message = get_app_client_pair(pair,nn_node,cc_node)
                if result == True:
                    return result, message

                # Return back to current depth
                pair.remove(p)
                nn_node.append(n)
                cc_node[c] = c_node[c]

    return result, message

def is_valid_pair(n,c):
    client = node = [0,1,2]
    node[0], node[1], node[2], app, name = n
    client[0], client[1], client[2], apps, gatewayname = c

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


def create_client_dict(client_list):
    c_node = {}
    for c in client_list:

        apps = [ a.name for a in c.apps]
        c_node[c.name]= (c.platform, c.system, c.robot,apps, c.gateway_name)

    return c_node

