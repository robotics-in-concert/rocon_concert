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
import concert_msgs.srv as concert_srvs

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
        rospy.wait_for_service('start_solution')
        self._start_solution = rospy.ServiceProxy('start_solution', concert_srvs.StartSolution)

    def _callback_concert_clients(self, concert):
        '''
          The conductor publishes the concert client list, which also happens to
          be latched so you'll always get the latest list.
        '''
        self._concert_clients = copy.deepcopy(concert.clients)
        rospy.loginfo("Orchestration : updated concert clients list:")
        for concert_client in concert.clients:
            rospy.loginfo("       Client: %s" % (concert_client.name))
            rospy.loginfo("               %s.%s.%s" % (concert_client.platform, concert_client.system, concert_client.robot))
            rospy.loginfo("               %s" % concert_client.client_status)
        if not self._solution_running:
            node_client_matches = self._implementation_ready()
            if node_client_matches:
                self._implementation.rebuild(node_client_matches)
                self._implementation.publish()
                try:
                    req = concert_srvs.StartSolutionRequest()
                    req.implementation = self._implementation.to_msg()
                    unused_resp = self._start_solution(req)
                except rospy.ServiceException, e:
                    rospy.logwarn("Orchestration : service call failed [%s]" % e)

    def _implementation_ready(self):
        '''
          Checks if the listed concert clients are a match with the
          implementation.

          @return list of (node, client) tuples
        '''
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
                return None
            elif len(possible_match_indices) == 1:
                matched.append((node['id'], clients[possible_match_indices[0]].name))
                del clients[possible_match_indices[0]]
            else:
                matching_index = possible_match_indices[0]
                for index in possible_match_indices:
                    if node['id'] == clients[index].name:
                        matching_index = index
                        break
                matched.append((node['id'], clients[matching_index].name))
                #print "Appending matched %s-%s" % (node['id'], clients[matching_index].name)
                del clients[matching_index]
        return matched

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
