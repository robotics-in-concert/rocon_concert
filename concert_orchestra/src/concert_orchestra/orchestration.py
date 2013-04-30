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

##############################################################################
# Orchestration
##############################################################################


class Orchestration(object):

    def __init__(self):
        self._implementation = Implementation()
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

          @param up to date concert client list provided by the conductor
          @type concert_msgs.ConcertClients
        '''
        rospy.loginfo("Orchestration : updated concert clients list:")
        self._concert_clients = {}  # maybe small race condition in doing this
        for concert_client in concert.clients:
            # create a dictionary of concert client objects, keyed by the human consumable name
            self._concert_clients[concert_client.name] = copy.deepcopy(concert_client)
            rospy.loginfo("       Client: %s" % (concert_client.name))
            rospy.loginfo("               %s.%s.%s" % (concert_client.platform, concert_client.system, concert_client.robot))
            rospy.loginfo("               %s" % concert_client.client_status)
        node_client_matches = self._implementation_ready()
        if node_client_matches:
            if not self._solution_running:
                self._implementation.rebuild(node_client_matches)
                self._implementation.publish()
                rospy.loginfo("Orchestration : solution is ready to run")
                if self._params['auto_start']:
                    self._process_start_solution(concert_srvs.StartSolutionRequest())
        else:
            # means you've lost a client
            # probably not robust if you have apps coming and going
            self._solution_running = False
            #self._process_stop_solution()

    def _implementation_ready(self):
        '''
          Checks if the listed concert clients are a match with the
          implementation.

          @return list of (node, client) tuples or None
        '''
        clients = copy.deepcopy(self._concert_clients.values())
        compatibles = []
        # make a list of (node, matching client list) tuples
        for node in self._implementation.nodes:
            print "Node %s" % str(node)
            compatible_clients = [client for client in clients if self._compatible_node_client(node, client)]
            print "  Matching Clients: %s" % str([client.name for client in compatible_clients])
            compatibles.append((node, compatible_clients))


        return matched

    def _perfect_matches(self, node_client_list_pairs):
        '''
          Checks a (node, matching_client[]) tuple list to see if nodes and client names
          match and min/max conditions are satisfied.

          Names will successivly match if there's only a trailing numerical number difference.

          WARNING: this is only valid if we assume the nodes are uniquely named in the implementation.

          @param node_client_list_pairs
          @type [ (concert_msgs.LinkNode, concert_msgs.ConcertClient[]) ]

          @return node - client list (pruned of non-perfect matches)
          @rtype [ (concert_msgs.LinkNode, concert_msgs.ConcertClient[]) ]
        '''
        node_perfect_match_client_list_pairs = []
        for (node, compatible_client_list) in node_client_list_pairs:
            # matches for which the names only differ by a trailing numerical value (e.g. dude, dude1234)
            perfect_matches = [client for client in compatible_client_list if re.match(node.name, re.sub('[0-9]*$', '', client.name))]
            node_perfect_match_client_list_pairs.append((node, perfect_matches))
        # Check min quota for each node




    def _compatible_node_client(self, node, concert_client):
        '''
          Checks to see if a client is compatible for the implementation's node rule.
        '''
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

    ##########################################################################
    # Ros Callbacks
    ##########################################################################
    # These should be moved to the conductor under the guise of
    # 'start apps', 'stop apps' (plural).

    def _process_start_solution(self, req):
        # Put in checks to see if a solution is already running
        response = concert_srvs.StartSolutionResponse()
        if not self._implementation_ready():
            response.success = False
            response.message = "solution is not yet ready (waiting for clients)..."
            return response
        if self._solution_running:
            rospy.loginfo("Orchestration : chincha? the solution is already running, try restarting anyway")
        implementation = self._implementation.to_msg()
        response.success = True
        response.message = "bonza"
        link_graph = implementation.link_graph
        rospy.loginfo("Orchestra : starting solution [%s]" % implementation.name)
        for node in link_graph.nodes:
            concert_client_name = node.id
            app_name = node.tuple.split('.')[3]
            remappings = []
            rospy.loginfo("            node: %s" % concert_client_name)
            rospy.loginfo("              app: %s" % app_name)
            rospy.loginfo("              remaps")
            for edge in link_graph.edges:
                if edge.start == concert_client_name or edge.finish == concert_client_name:
                    rospy.loginfo("                %s->%s" % (edge.remap_from, edge.remap_to))
                    remappings.append((edge.remap_from, edge.remap_to))
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

    def _process_stop_solution(self, req=None):
        response = concert_srvs.StopSolutionResponse()
        response.success = True
        response.message = "Bonza"
        if not self._solution_running:
            response.success = False
            response.message = "chincha? the solution is not running..."
            return response
        self._solution_running = False
        rospy.loginfo("Orchestra : stopping the solution.")
        for node in self._implementation.nodes:
            stop_app_name = '/' + node['id'] + '/stop_app'
            app_name = node['tuple'].split('.')[3]
            # check first if it exists, also timeouts?
            rospy.wait_for_service(stop_app_name)
            stop_app = rospy.ServiceProxy(stop_app_name, rapp_manager_srvs.StopApp)
            req = rapp_manager_srvs.StopAppRequest(app_name)
            resp = stop_app(req)
            if not resp.stopped:
                response.success = False
                response.message = "aigoo, failed to stop app %s" % app_name
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
