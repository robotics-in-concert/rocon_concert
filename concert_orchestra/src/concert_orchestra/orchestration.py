#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/hydro-devel/concert_orchestra/LICENSE
#
##############################################################################
# Imports
##############################################################################

import re
import copy
import rospy
import rocon_app_manager_msgs.msg as rapp_manager_msgs
import rocon_app_manager_msgs.srv as rapp_manager_srvs
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs

# Local imports
from .implementation import Implementation
from .compatibility_tree import create_compatibility_tree, prune_compatibility_tree, CompatibilityTree

##############################################################################
# Orchestration
##############################################################################


class Orchestration(object):

    def __init__(self):
        self._implementation = Implementation()
        self._compatibility_tree = create_compatibility_tree(self._implementation.nodes, {})
        self._solution_running = False
        self._concert_clients = {}  # dictionary of human friendly name - concert_msgs.ConcertClient pairs
        rospy.Subscriber("list_concert_clients", concert_msgs.ConcertClients, self._callback_concert_clients)

        # parameters
        self._params = {}
        self._params['auto_start'] = rospy.get_param("~auto_start", False)

        self._services = {}
        # later disassemble these to start_apps/stop_apps (plural) to the conductor
        self._services['stop_solution'] = rospy.Service('stop_solution', concert_srvs.StopSolution, self._process_stop_solution)
        self._services['start_solution'] = rospy.Service('start_solution', concert_srvs.StartSolution, self._process_start_solution)

    def _callback_concert_clients(self, concert):
        '''
          The conductor publishes the concert client list, which also happens to
          be latched so you'll always get the latest list.

          It stores the concert clients in a dictionary of concert clients keyed
          by the client names.

          @todo - write what is the imperative reason for a dic over a simple list

          @param up to date concert client list provided by the conductor
          @type concert_msgs.ConcertClients
        '''
        rospy.loginfo("Orchestration : updated concert clients list:")
        old_concert_clients = copy.deepcopy(self._concert_clients)
        self._concert_clients = {}  # maybe small race condition in doing this
        for concert_client in concert.clients:
            # create a dictionary of concert client objects, keyed by the human consumable name
            self._concert_clients[concert_client.name] = copy.deepcopy(concert_client)
            rospy.loginfo("       Client: %s" % (concert_client.name))
            rospy.loginfo("               %s.%s.%s" % (concert_client.platform, concert_client.system, concert_client.robot))
            rospy.loginfo("               %s" % concert_client.client_status)
        if not self._solution_running:
            self._compatibility_tree = create_compatibility_tree(self._implementation.nodes, self._concert_clients)
            pruned_branches = prune_compatibility_tree(self._compatibility_tree, verbosity=True)
            if pruned_branches:
                self._pruned_compatibility_tree = CompatibilityTree(pruned_branches)
                if self._pruned_compatibility_tree.is_valid():
                    rospy.loginfo("Orchestration : solution is ready to run")
                    # Could do this, but no-one is listening right now.
                    # self._implementation.rebuild(node_client_matches)
                    # self._implementation.publish()
                    if self._params['auto_start']:
                        self._process_start_solution(concert_srvs.StartSolutionRequest())
                else:
                    rospy.loginfo("Orchestration : solution not yet ready [%s]" % self._pruned_compatibility_tree.error_message)
                    self._pruned_compatibility_tree.print_branches("Current Branches")
        else:
            diff = lambda l1, l2: [x for x in l1 if x not in l2]  # diff of lists
            new_client_names = diff(self._concert_clients.keys(), old_concert_clients.keys())
            lost_client_names = diff(old_concert_clients.keys(), self._concert_clients.keys())
            for new_client_name in new_client_names:
                new_client = self._concert_clients[new_client_name]
                branch = self._pruned_compatibility_tree.add_leaf(new_client)
                if branch is None:
                    rospy.logerr("Orchestra: this client is not compatible for any node in this implementation.")
                    # should get listed as a bad client?
                else:
                    self._process_start_client(new_client, branch)
            for lost_client_name in lost_client_names:
                lost_client = old_concert_clients[lost_client_name]
                self._pruned_compatibility_tree.remove_leaf(lost_client)
                if not self._pruned_compatibility_tree.is_valid():
                    rospy.logerr("Orchestra: client disengaged and the solution is now no longer valid [%s]" % lost_client_name)
                    self._process_stop_solution()

    ##########################################################################
    # Ros Callbacks
    ##########################################################################
    # These should be moved to the conductor under the guise of
    # 'start apps', 'stop apps' (plural).

    def _process_start_solution(self, req):
        # Put in checks to see if a solution is already running
        response = concert_srvs.StartSolutionResponse()
        if not self._pruned_compatibility_tree.is_valid():
            response.success = False
            response.message = "cowardly refused to start the solution [%s]..." % self._pruned_compatibility_tree.error_message
            rospy.loginfo("Orchestration : %s" % response.message)
            self._pruned_compatibility_tree.print_branches()
            return response
        if self._solution_running:
            rospy.logwarn("Orchestration : %s" % response.message)
            response.message = "chincha? the solution is already running..."
            response.success = False
            return response
        implementation = self._implementation.to_msg()
        response.success = True
        response.message = "bonza"
        link_graph = implementation.link_graph

        rospy.loginfo("Orchestra : starting solution [%s]" % implementation.name)
        for branch in self._pruned_compatibility_tree.branches:
            app_name = branch.node.tuple.split('.')[3]
            node_name = branch.node.id
            remappings = []
            for edge in link_graph.edges:
                if edge.start == node_name or edge.finish == node_name:
                    remappings.append((edge.remap_from, edge.remap_to))
            for leaf in branch.leaves:
                concert_client_name = leaf.name
                rospy.loginfo("            node: %s/%s" % (node_name, concert_client_name))
                rospy.loginfo("              app: %s" % app_name)
                rospy.loginfo("              remaps")
                for (remap_from, remap_to) in remappings:
                    rospy.loginfo("                %s->%s" % (remap_from, remap_to))
                # Check to see if start app service exists for the node, abort if not
                start_app_name = '/' + self._concert_clients[concert_client_name].gateway_name + '/start_app'
                rospy.wait_for_service(start_app_name)
                start_app = rospy.ServiceProxy(start_app_name, rapp_manager_srvs.StartApp)
                req = rapp_manager_srvs.StartAppRequest()
                req.name = app_name
                req.remappings = []
                for remapping in remappings:
                    req.remappings.append(rapp_manager_msgs.Remapping(remapping[0], remapping[1]))
                rospy.loginfo("              Starting...")
                resp = start_app(req)
                if not resp.started:
                    response.success = False
                    response.message = "aigoo, failed to start app %s of %s" % (app_name, concert_client_name)
                    rospy.logwarn("              failed to start app %s" % (app_name))
        if response.success:
            rospy.loginfo("Orchestra: All clients' app are started")
        else:
            rospy.logwarn("Orchestra: " + response.message)

        self._solution_running = True
        return response

    def _process_start_client(self, client, branch):
        '''
          Used to start a single client. This is done when a client dynamically joins after the solution has started.
        '''
        app_name = branch.node.tuple.split('.')[3]
        node_name = branch.node.id
        remappings = []
        implementation = self._implementation.to_msg()
        link_graph = implementation.link_graph
        for edge in link_graph.edges:
            if edge.start == node_name or edge.finish == node_name:
                remappings.append((edge.remap_from, edge.remap_to))
        rospy.loginfo("            node: %s/%s" % (node_name, client.name))
        rospy.loginfo("              app: %s" % app_name)
        rospy.loginfo("              remaps")
        for (remap_from, remap_to) in remappings:
            rospy.loginfo("                %s->%s" % (remap_from, remap_to))
        # Check to see if start app service exists for the node, abort if not
        start_app_name = '/' + client.gateway_name + '/start_app'
        rospy.wait_for_service(start_app_name)
        start_app = rospy.ServiceProxy(start_app_name, rapp_manager_srvs.StartApp)
        req = rapp_manager_srvs.StartAppRequest()
        req.name = app_name
        req.remappings = []
        for remapping in remappings:
            req.remappings.append(rapp_manager_msgs.Remapping(remapping[0], remapping[1]))
        rospy.loginfo("              Starting...")
        resp = start_app(req)
        if not resp.started:
            rospy.logwarn("              failed to start app %s" % (app_name))

    def _process_stop_solution(self, req=None):
        response = concert_srvs.StopSolutionResponse()
        response.success = True
        response.message = "Bonza"
        if not self._solution_running:
            response.success = False
            response.message = "chincha? the solution is not running..."
            rospy.logwarn("Orchestration : %s" % response.message)
            return response
        self._solution_running = False
        rospy.loginfo("Orchestra : stopping the solution.")
        for branch in self._pruned_compatibility_tree.branches:
            app_name = branch.node.tuple.split('.')[3]
            for leaf in branch.leaves:
                stop_app_name = '/' + leaf.gateway_name + '/stop_app'
                rospy.loginfo("Orchestra :   stopping %s" % stop_app_name)
                # check first if it exists, also timeouts?
                rospy.wait_for_service(stop_app_name)
                stop_app = rospy.ServiceProxy(stop_app_name, rapp_manager_srvs.StopApp)
                req = rapp_manager_srvs.StopAppRequest()
                resp = stop_app(req)
                if not resp.stopped:
                    response.success = False
                    response.message = "aigoo, failed to stop app %s" % app_name
	rospy.loginfo("Orchestra : the solution has stopped successfully")
        return response

##############################################################################
# Graveyard
##############################################################################

#    def _implementation_ready_graveyard(self):
#        '''
#          Checks if the listed concert clients are a match with the
#          implementation.
#
#          @return list of (node, client) tuples or None
#        '''
#        clients = copy.deepcopy(self._concert_clients.values())
#        matched = []
#        for node in self._implementation.nodes:
#            #print "Node %s" % str(node)
#            index = 0
#            possible_match_indices = []
#            for client in clients:
#                if self._match(node, client):
#                    possible_match_indices.append(index)
#                index += 1
#            #print "Possible match indices %s" % str(possible_match_indices)
#            if not possible_match_indices:
#                #print "Match failed: %s" % str(node)
#                return None
#            elif len(possible_match_indices) == 1:
#                matched.append((node['id'], clients[possible_match_indices[0]].name))
#                del clients[possible_match_indices[0]]
#            else:
#                matching_index = possible_match_indices[0]
#                for index in possible_match_indices:
#                    if node['id'] == clients[index].name:
#                        matching_index = index
#                        break
#                matched.append((node['id'], clients[matching_index].name))
#                #print "Appending matched %s-%s" % (node['id'], clients[matching_index].name)
#                del clients[matching_index]
#        return matched
