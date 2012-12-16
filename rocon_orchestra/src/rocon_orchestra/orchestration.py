#!/bin/env python

'''
The orchestrator.

@author: Daniel Stonier
'''
##############################################################################
# Imports
##############################################################################

import copy
import roslib
roslib.load_manifest('rocon_orchestra')
import rospy
import concert_msgs.msg as concert_msgs

# Local imports
from .implementation import Implementation

##############################################################################
# Callbacks
##############################################################################


class Orchestration(object):

    def __init__(self):
        self._implementation = Implementation()
        self._solution_running = False
        self._concert_clients = []
        rospy.Subscriber("list_concert_clients", concert_msgs.ConcertClients, self._callback_concert_clients)

    def _callback_concert_clients(self, concert):
        '''
          The conductor publishes the concert client list, which also happens to
          be latched so you'll always get the latest list.
        '''
        self._concert_clients = copy.deepcopy(concert.clients)
        for concert_client in concert.clients:
            rospy.loginfo("Orchestration: updated concert clients list:")
            rospy.loginfo("       Client: %s" % (concert_client.name))
            rospy.loginfo("               %s.%s.%s" % (concert_client.platform, concert_client.system, concert_client.robot))
            rospy.loginfo("               %s" % concert_client.client_status)
        if not self._solution_running:
            if self._implementation_ready():
                print "********** Ready **************"

    def _implementation_ready(self):
        clients = copy.deepcopy(self._concert_clients)
        matched = []
        for node in self._implementation.nodes:
            #print "Node %s" % str(node)
            index = 0
            possible_match_indices = []
            for client in clients:
                if self._match(node, client):
                    possible_match_indices.append(index)
                index += 1
            #print "Possible match indices %s" % str(possible_match_indices)
            if not possible_match_indices:
                #print "Match failed: %s" % str(node)
                return False
            elif len(possible_match_indices) == 1:
                matched.append((copy.deepcopy(node), clients[possible_match_indices[0]]))
                del clients[possible_match_indices[0]]
            else:
                matching_index = possible_match_indices[0]
                for index in possible_match_indices:
                    if node['id'] == clients[index].name:
                        matching_index = index
                        break
                matched.append((copy.deepcopy(node), copy.deepcopy(clients[matching_index])))
                #print "Appending matched %s-%s" % (node['id'], clients[matching_index].name)
                del clients[matching_index]
        return True

    def _match(self, node, concert_client):
        #print "****** _match ******"
        #print str(node)
        #print concert_client.name + "-" + concert_client.platform + "." + concert_client.system + "." + concert_client.robot
        parts = node['tuple'].split('.')
        platform = parts[0]
        system = parts[1]
        robot = parts[2]
        app_name = parts[3]
        if platform != concert_client.platform:
            return False
        if system != concert_client.system:
            return False
        if robot != concert_client.robot:
            return False
        for client_app in concert_client.apps:
            if app_name == client_app.name:
                return True
        return False
