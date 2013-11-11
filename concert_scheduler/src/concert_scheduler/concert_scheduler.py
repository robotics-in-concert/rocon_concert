#!/usr/bin/env python

import rospy
import threading
import copy
import concert_msgs.srv as concert_srv
import concert_msgs.msg as concert_msg
import rocon_std_msgs.msg as rocon_std_msg
import rocon_app_manager_msgs.srv as rapp_mamanager_srvs

import compatibility_tree
from .exceptions import *

class ConcertScheduler(object):

    POSTFIX_START_APP = '/start_app'
    POSTFIX_STOP_APP = '/stop_app'

    def __init__(self):
        self._init_variables()
        self.setup_ros_api()
        self.lock = threading.Lock()
        rospy.on_shutdown(self.shutdown)

    def init_variables(self):
        """
            Initialize variables
        """
        self.sub = {}
        self.srv = {}

        self.services = {}
        self.clients = {}
        self.inuse = {}
        self.pairs = {}
        self.lock = None

    def setup_ros_api(self):
        self.sub['list_concert_clients'] = rospy.Subscriber('list_concert_clients', concert_msg.ConcertClients, self.process_list_concert_clients)
        self.sub['request_resources'] = rospy.Subscriber('request_resources', concert_msg.RequestResources, self.process_request_resources)

        self.srv['resource_status'] = rospy.Service('resource_status', concert_srv.ResourceStatus, self.process_resource_status)

    def shutdown(self):
        """
            to clean up concert scheduler. It stops all clients' app 
        """
        pass

    def process_list_concert_clients(self, msg):
        """
            @param
                msg : concert_msg.ConcertClients

            1. Stops services which client left.
            2. Rebuild client list
            3. Starts services which has all requested resources
        """
        self.lock.acquire()

        clients = msg.clients

        self._stop_services_of_left_clients(clients)

        self.clients = {}
        for c in clients:
            self.clients[c.name] = c

        self._update_services_status()
        self.lock.release()

    def process_request_resources(self, msg):
        """
            1. enable : true. add service or update linkgraph
            2. enable : false. stop service. and remove service
            3. Starts services which has all requested resources

            TODO: Current implementation expects service_name to be unique.
            TODO: it does not rearrange clients if only linkgraph get changed.

            @param : msg
            @type concert_msg.RequestResources
        """
        self.loginfo("Received request")
        self.lock.acquire()

        if msg.enable is True:
            self.services[msg.service_name] = msg.linkgraph
        else:
            if msg.service_name in self.pairs:
                self._stop_service(msg.service_name)
            if msg.service_name in self.services:
                del self.services[msg.service_name]
            else:
                self.loginfo("[" + str(msg.service_name) + "] does not exist. Available Service " + str(self.services.keys()))

        self._update_services_status()

        self.lock.release()

    def _stop_services_of_left_clients(self, clients):
        """
            1. Get gateway_name of clients which left concert
            2. Check all pairs of services whether it is still valid.
            3. Stops services which left_clients are involved

            @param clients : list of client who recently left. list of concert_msg.ConcertClient
            @type concert_msg.ConcertClient[]
        """

        left_client_gateways = self._get_left_clients(clients)
        service_to_stop = []

        for service_name in self.pairs:
            if not self._is_service_still_valid(self.pairs[service_name], left_client_gateways):
                service_to_stop.append(service_name)

        for s in service_to_stop:
            self._stop_service(s, left_client_gateways)

    def _is_service_still_valid(self, pairs, left_clients):
        """
            @param pair: list of pair (service node, client node)
            @type [(concert_msg.LinkNode, concert_msg.ConcertClient)]

            @param left_clients: list of concert client gateways who recently left
            @type string[]
        """
        g_set = set([client_node.gateway_name for _1, client_node in pairs])
        l_set = set(left_clients)

        intersect = g_set.intersection(l_set)
        if len(intersect) > 0:
            return False
        else:
            return True

    def _stop_service(self, service_name, left_client_gateways=[]):
        """
            Stops all clients involved in <service_name>
        """
        for p in self.pairs[service_name]:
            service_node, client_node = p

            service_node_name = service_node.id
            _1, _2, _3, _4, service_app_name = service_node.tuple.split(".")

            self.loginfo("Node : " + str(service_node_name) + "\tApp : " + str(service_app_name) + "\tClient : " + str(client_node.name))

            is_need_to_stop = self._unmark_client_as_inuse(client_node.gateway_name)

            if not client_node.gateway_name in left_client_gateways:
                if is_need_to_stop:  # only if nobody is using the client
                    # Creates stop app service
                    stop_app_srv = self._get_stop_client_app_service(client_node.gateway_name)

                    # Create stop app request object
                    req = rapp_mamanager_srvs.StopAppRequest()

                    # Request
                    self.loginfo("    Stopping...")
                    resp = stop_app_srv(req)

                    if not resp.stopped:
                        message = "Failed to stop[" + str(service_app_name) + "] in [" + str(client_node.name) + "]"
                        raise Exception(message)
                    else:
                        self.loginfo("    Done")
            else:
                self.loginfo(str(client_node.name) + " has already left")

        del self.pairs[service_name]
        self.loginfo(str(service_name) + " has been stopped")

    def _get_stop_client_app_service(self, gatewayname):
        """
            @param gateway_name: The client gateway name
            @type string

            @return srv: ROS service instance to stop app
            @type rospy.ServiceProxy
        """
        srv_name = '/' + gatewayname + ConcertScheduler.POSTFIX_STOP_APP
        rospy.wait_for_service(srv_name)

        srv = rospy.ServiceProxy(srv_name, rapp_mamanager_srvs.StopApp)

        return srv

    def _get_left_clients(self, clients):
        """
            @param clients: list of concert client
            @type concert_msg.ConcertClient[]

            @return left_clients: list of concert client gateway_name who recently left
            @rtype list of string
        """

        inuse_gateways = self._get_inuse_gateways()
        inuse_gateways_set = set(inuse_gateways)
        clients_set = set([c.gateway_name for c in clients])
        left_clients = inuse_gateways_set.difference(clients_set)

        return list(left_clients)

    def _update_services_status(self):
        """ 
            Brings up services which can start with currently available clients(or resources)

            For each service which needs to be started
            1. create (node, client) pair with already running clients.
            2. create (node, client) pair with available clients
            3. If all pairs are successfully made, starts service

            TODO : Current implementation is not aware of priority of service
        """
        for s in self.services:
            linkgraph = self.services[s]

            if s in self.pairs:
                # nothing to touch
                continue

            # reuse running clients as much as possible
            nodes = copy.deepcopy(linkgraph.nodes)
            app_pairs = []


            available_clients = self._get_available_clients()
            cname = [c.name for c in available_clients]
            self.loginfo("Avaialble client : " + str(cname))

            rn = [(n.id, n.tuple) for n in nodes] 
            self.loginfo("Remaining node : " + str(rn))

            # Check if any node can be paired with already running client
            paired_nodes, reused_client_app_pairs = self._pairs_with_running_clients(nodes, linkgraph)
            compatibility_tree.print_pairs(reused_client_app_pairs)

            # More client is required?
            remaining_nodes = [n for n in nodes if not n in paired_nodes]

            # Clients who are not running yet
            available_clients = self._get_available_clients()
            cname = [c.name for c in available_clients]
            self.loginfo("Avaialble client : " + str(cname))

            rn = [n.id for n in remaining_nodes] 
            self.loginfo("Remaining node : " + str(rn))

            # Pair between remaining node and free cleints
            status, message, new_app_pairs = compatibility_tree.resolve(remaining_nodes, available_clients)

            app_pairs.extend(reused_client_app_pairs)
            app_pairs.extend(new_app_pairs)

            # Starts service if it is ready
            if status is concert_msg.ConcertService.READY:
                self.pairs[s] = app_pairs
                self._mark_clients_as_inuse(app_pairs, linkgraph)
                self._start_service(new_app_pairs, str(s), linkgraph)
            else:
                # if not, do nothing
                self.logwarn("Warning in [" + str(s) + "] status. " + str(message))

    def _pairs_with_running_clients(self, service_nodes, linkgraph):
        """
            Check if clients already running can be resued for another service nope
            To be compatibile, it should have the same remapping rules, and available slot.

            @param service_nodes: list of service nodes
            @type concert_msg.LinkNode[]

            @param linkgraph: implementation of service
            @type concert_msg.LinkGraph

            @return paired_node: which can reuse inuse client
            @type concert_msg.LinkNode[]

            @return new_app_pair: new pair
            @type [(concert_msg.LinkNode, concert_msg.ConcertClient)]
        """
        paired_node = []
        new_app_pairs = []

        for n in service_nodes:
            service_node_name = n.id
            _1, _2, _3, _4, service_app_name = n.tuple.split(".")

            if service_app_name in self.inuse:
                service_remappings = self._get_match_remappings(service_node_name, linkgraph.edges)

                for client_name, tup in self.inuse[service_app_name].items():
                    client_node, remappings, max_share, count = tup

                    if remappings == service_remappings and count < max_share:
                        paired_node.append(n)
                        new_app_pairs.append((n, client_node))

        return paired_node, new_app_pairs

    def _mark_clients_as_inuse(self, pairs, linkgraph):
        """
            store the paired client in inuse_clients

            @param pair: list of pair (service node, client node)
            @type : list of (concert_msg.LinkNode, concert_msg.ConcertClient)

            @param linkgraph: service linkgraph
            @type : concert_msg.LinkGraph
        """
        compatibility_tree.print_pairs(pairs)
        edges = linkgraph.edges

        for p in pairs:
            service_node, client_node = p

            _1, _2, _3, _4, service_app_name = service_node.tuple.split(".")
            remappings = self._get_match_remappings(service_node.id, linkgraph.edges)

            if not service_app_name in self.inuse:
                self.inuse[service_app_name] = {}

            app = [a for a in client_node.apps if a.name == service_app_name][0]  # must be only one

            if client_node.name in self.inuse[service_app_name]:
                c, r, m, count = self.inuse[service_app_name][client_node.name]
                self.inuse[service_app_name][client_node.name] = (c, r, m, count + 1)
            else:
                self.inuse[service_app_name][client_node.name] = (client_node, remappings, app.share, 1)

    def _unmark_client_as_inuse(self, gateway_name):
        """
            mark that client is not inuse.

            @param client_gateway_name
            @type string

            @return True if no service is using this client. so it stops the app
            @rtype bool
        """

        for app_name, client_node_dict in self.inuse.items():
            for client_node_name, tup in client_node_dict.items():
                client_node, remapping, max_share, count = tup

                if client_node.gateway_name == gateway_name:
                    if count > 1:
                        client_node_dict[client_node_name] = (client_node, remapping, max_share, count - 1)
                        return False
                    else:
                        client_node_dict[client_node_name] = (client_node, remapping, max_share, count - 1)
                        del self.inuse[app_name][client_node_name]
                        return True

    def _get_available_clients(self):
        """
            Search clients which are not in use yet.

            @return list of available clients
            @rtype concert_msg.ConcertClient[]
        """
        clients = [self.clients[c] for c in self.clients if self.clients[c].client_status == concert_msg.Constants.CONCERT_CLIENT_STATUS_CONNECTED]  # and self.clients[c].app_status == concert_msg.Constants.APP_STATUS_STOPPED]
        inuse_gateways = self._get_inuse_gateways()
        free_clients = [c for c in clients if not (c.gateway_name in inuse_gateways)]

        return free_clients

    def _get_inuse_gateways(self):
        """
            Iterate nested inuse and make a list of gateways

            @return list of gateway_name
            @rtype string[]
        """
        inuse_gateways = []

        for client_node_dict in self.inuse.values():
            for client_node, remapping, max_share, count in client_node_dict.values():
                if count > 0:
                    inuse_gateways.append(client_node.gateway_name)

        return inuse_gateways

    def _start_service(self, pairs, service_name, linkgraph):
        """
            Starts up client apps for service

            @param pairs: list of pair for service
            @type [(concert_msg.LinkNode, concert_msg.ConcertClient)]

            @param service_name
            @type string

            @param linkgraph: for service remapping
            @type concert_msg.LinkGraph
        """
        self.loginfo("Starting apps for " + str(service_name))

        for p in pairs:
            service_node, client_node = p

            service_node_name = service_node.id
            _1, _2, _3, _4, service_app_name = service_node.tuple.split(".")

            self.loginfo("Node : " + str(service_node_name) + "\tApp : " + str(service_app_name) + "\tClient : " + str(client_node.name))

            # Creates start app service
            start_app_srv = self._get_start_client_app_service(client_node.gateway_name)

            # Create start app request object
            req = self._create_startapp_request(service_app_name, service_node_name, linkgraph, service_name)

            # Request client to start app
            self.loginfo("    Starting...")
            self.loginfo(str(req.remappings))
            resp = start_app_srv(req)

            if not resp.started:
                message = resp.message
                raise FailedToStartAppsException(message)
            else:
                self.loginfo("    Done")
        self.loginfo(str(service_name) + " has been started")

    def _get_start_client_app_service(self, gateway_name):
        """
            @param gateway_name: The client gateway name
            @type string

            @return srv: ROS service instance to start app
            @type rospy.ServiceProxy
        """
        srv_name = '/' + gateway_name + ConcertScheduler.POSTFIX_START_APP
        rospy.wait_for_service(srv_name)

        srv = rospy.ServiceProxy(srv_name, rapp_mamanager_srvs.StartApp)

        return srv

    def _create_startapp_request(self, service_app_name, service_node_name, linkgraph, service_name):
        """
            creates startapp request instance

            @param service_app_name: name of app to start in client
            @type string

            @param service_node_name
            @type string

            @param linkgraph
            @type concert_msg.LinkGraph

            @param service_name
            @type string
        """

        req = rapp_mamanager_srvs.StartAppRequest()
        req.name = service_app_name

        # Remappings
        edges = linkgraph.edges
        req.remappings = self._get_match_remappings(service_node_name, edges)

        return req

    def _get_match_remappings(self, node_name, edges):
        """
            finds out matching remappings for given node

            @param node_name
            @type string

            @param edges
            @type concert_msg.LinkEdge[]
        """

        remappings = [rocon_std_msg.Remapping(e.remap_from, e.remap_to) for e in edges if e.start == node_name or e.finish == node_name]
        return remappings

    def process_resource_status(self, req):
        """
            respond resource_status request
        """
        resp = concert_srv.ResourceStatusResponse()

        self.lock.acquire()

        inuse_gateways = self._get_inuse_gateways()
        available = [c for c in self.clients if not self.clients[c].gateway_name in inuse_gateways]

        inuse = []
        for client_node_dict in self.inuse.values():
            for client_node, remapping, max_share, count in client_node_dict.values():
                s = client_node.name + " - " + str(count) + "(" + str(max_share) + ")"
                inuse.append(s)

        resp.available_clients = available
        resp.inuse_clients = inuse

        resp.requested_resources = []
        for s in self.services:
            srp = concert_msg.ServiceResourcePair()

            srp.service_name = s

            srp.resources = []
            for n in self.services[s].nodes:
                for i in range(n.min):
                    srp.resources.append(n.id)

            resp.requested_resources.append(srp)

        resp.engaged_pairs = []
        for s in self.pairs:
            srp = concert_msg.ServiceResourcePair()
            srp.service_name = s

            for pair in self.pairs[s]:
                service_node, client_node = pair

                service_node_name = service_node.id
                _1, _2, _3, _4, service_app_name = service_node.tuple.split(".")

                p = service_node_name + " - " + client_node.name + " - " + service_app_name

                srp.resources.append(p)

            resp.engaged_pairs.append(srp)

        self.lock.release()

        return resp

    def loginfo(self, msg):
        rospy.loginfo("Scheduler : " + str(msg))

    def logwarn(self, msg):
        rospy.logwarn("Scheduler : " + str(msg))

    def logerr(self, msg):
        rospy.logerr("Scheduler : " + str(msg))

    def spin(self):
        self.loginfo("In spin")
        rospy.spin()
