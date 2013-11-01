#!/usr/bin/env python

import rospy
import copy
import concert_msgs.msg as concert_msg

class CompatibilityTree(object):

    node = [] 

    def __init__(self,nodes):
        '''
            max is not considered.     
        '''

        self.node = []
        for n in nodes:
            platform, system, robot, app = n.tuple.split(".")

            for i in range(n.min):
                self.node.append((platform, system, robot, app, n.id))
            
    def is_ready(self,client_list): 

        # create app indexed dict
        c_node = self.create_client_dict(client_list)

#        rospy.loginfo("=== Clients ===")
#        self.print_dict(c_node)
#        rospy.loginfo("=== Service ===")
#        self.print_array(self.node)

        pair = []
        result, message = self.get_app_client_pair(pair,self.node,c_node)

#        rospy.loginfo("== Result")
#        self.print_array(pair)
#        rospy.loginfo("================")
        return result, message, pair

    def get_app_client_pair(self,pair,n_node,c_node):
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
                if self.is_valid_pair(n,c_node[c]):
                    _1,_2,_3, _4, gatewayname = c_node[c]
                    p = (n, c, gatewayname)

                    # Prepare for next depth
                    pair.append(p)
                    nn_node.remove(n)
                    del cc_node[c]

                    # Go to next depth
                    result, message = self.get_app_client_pair(pair,nn_node,cc_node)
                    if result == concert_msg.ConcertService.READY:
                        return result, message

                    # Return back to current depth
                    pair.remove(p)
                    nn_node.append(n)
                    cc_node[c] = c_node[c]

        return result, message

    def is_valid_pair(self,n,c):
        client = node = [0,1,2]
        node[0], node[1], node[2], app, name = n
        client[0], client[1], client[2], apps, gatewayname = c

        for i in range(3):
            if not (node[i] == client[i] or node[i] == '*'):
                return False

        if not (app in apps):
            return False
        
        return True
                

    def print_dict(self,c_node):
        for c in c_node:
            rospy.loginfo(str(c) + " : " + str(c_node[c]))

    def print_array(self,pair):
        for p in pair:
            rospy.loginfo(str(p))


    def create_client_dict(self,client_list):
        c_node = {}
        for c in client_list:

            apps = [ a.name for a in c.apps]
            c_node[c.name]= (c.platform, c.system, c.robot,apps, c.gateway_name)

        return c_node

