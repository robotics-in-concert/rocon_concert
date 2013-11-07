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

    sub = {}
    srv = {}

    services = {}
    clients = {}
    inuse_clients = []
    pairs = {}

    postfix_start_app = '/start_app'
    postfix_stop_app = '/stop_app'

    lock = None

    def __init__(self):
        self.lock = threading.Lock()
        self.setup_ros_api()

    def setup_ros_api(self):
        self.sub['list_concert_clients'] = rospy.Subscriber('list_concert_clients', concert_msg.ConcertClients, self.process_list_concert_clients)
        self.sub['request_resources'] = rospy.Subscriber('request_resources', concert_msg.RequestResources, self.process_request_resources)

        self.srv['resource_status'] = rospy.Service('resource_status', concert_srv.ResourceStatus, self.process_resource_status)

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
            @param
                msg : concert_msg.RequestResources

            1. enable : true. add service or update linkgraph
            2. enable : false. stop service. and remove service
            3. Starts services which has all requested resources

            TODO: Current implementation expects service_name to be unique.
        """
        self.lock.acquire()

        if msg.enable is True:
            self.services[msg.service_name] = msg.linkgraph
        else:
            if msg.service_name in self.services:
                self._stop_service(msg.service_name)
                del self.services[msg.service_name]
            else:
                self.loginfo("[" + str(msg.service_name) + "] does not exist. Available Service " + str(self.services.keys()))

        self._update_services_status()

        self.lock.release()

    def _stop_services_of_left_clients(self, clients):
        """
            @param
                clients : list of concert_msg.ConcertClient

            1. Get gateway_name of clients which left concert
            2. Check all pairs of services whether it is still valid.
            3. Stops services which left_clients are involved
        """

        left_clients = self._get_left_clients(clients)
        service_to_stop = []

        for service_name in self.pairs:
            if not self._is_service_still_valid(self.pairs[service_name], left_clients):
                service_to_stop.append(service_name)

        for s in service_to_stop:
            self._stop_service(s, left_clients)

    def _is_service_still_valid(self, pairs, left_clients):
        """
            @param
        """
        g_set = set([gateway for _1, _2, gateway in pairs])
        l_set = set(left_clients)

        intersect = g_set.intersection(l_set)
        if len(intersect) > 0:
            return False
        else:
            return True

    def _stop_service(self, service_name, left_clients=[]):
        """
            Stops all clients involved in <service_name>
        """

        for pairs in self.pairs[service_name]:
            nodes, client, gateway_name = pairs
            _, __, ___, ____, app, node = nodes
            self.loginfo("Node : " + str(node) + "\tApp : " + str(app) + "\tClient : " + str(client))

            if not gateway_name in left_clients:

                # Creates stop app service
                stop_app_srv = self._get_stop_client_app_service(gateway_name)

                # Create stop app request object
                req = rapp_mamanager_srvs.StopAppRequest()

                # Request
                self.loginfo("    Stopping...")
                resp = stop_app_srv(req)

                if not resp.stopped:
                    message = "Failed to stop[" + str(app) + "] in [" + str(client) + "]"
                    raise Exception(message)
                else:
                    self.loginfo("    Done")
            else:
                self.loginfo("Client already left")

            self.inuse_clients.remove(gateway_name)

        del self.pairs[service_name]

    def _get_stop_client_app_service(self, gatewayname):
        srv_name = '/' + gatewayname + self.postfix_stop_app
        rospy.wait_for_service(srv_name)

        srv = rospy.ServiceProxy(srv_name, rapp_mamanager_srvs.StopApp)

        return srv

    def _get_left_clients(self, clients):

        inuse_gateway = set(self.inuse_clients)
        clients_set = set([c.gateway_name for c in clients])

        left_clients = inuse_gateway.difference(clients_set)

        return list(left_clients)

    def _update_services_status(self):
        """
            Current implementation is not aware of priority of service
        """
        for s in self.services:
            linkgraph = self.services[s]

            if s in self.pairs:
                # nothing to touch
                continue

            clients = self._get_available_clients()
            status, message, app_pairs = compatibility_tree.resolve(linkgraph.nodes, clients)

            self.loginfo(str(app_pairs))

            if status is concert_msg.ConcertService.READY:
                self.pairs[s] = app_pairs
                self._mark_clients_as_inuse(app_pairs)
                self._start_apps(self.pairs[s], str(s), linkgraph)
            else:
                self.logwarn("Warning in service status. " + str(message))

    def _mark_clients_as_inuse(self, pairs):
        for p in pairs:
            (_1, _2, gateway_name) = p
            self.inuse_clients.append(gateway_name)

    def _get_available_clients(self):
        clients = [self.clients[c] for c in self.clients if self.clients[c].client_status == concert_msg.Constants.CONCERT_CLIENT_STATUS_CONNECTED and self.clients[c].app_status == concert_msg.Constants.APP_STATUS_STOPPED]
        free_clients = [c for c in clients if not (c.gateway_name in self.inuse_clients)]

        return free_clients

    def _start_apps(self, pairs, service_name, linkgraph):
        self.loginfo("Starting apps for " + str(service_name))

        for nodes, client, client_gatewayname in pairs:
            _, __, ___, ____, app, node = nodes
            self.loginfo("Node : " + str(node) + "\tApp : " + str(app) + "\tClient : " + str(client))

            # Creates start app service
            start_app_srv = self._get_start_client_app_service(client_gatewayname)

            # Create start app request object
            req = self._create_startapp_request(app, node, linkgraph, service_name)

            # Request client to start app
            self.loginfo("    Starting...")
            self.loginfo(str(req.remappings))
            resp = start_app_srv(req)

            if not resp.started:
#                message = "Failed to start [" + str(app) + "] in [" + str(client) +"]"
                message = resp.message
                raise FailedToStartAppsException(message)
            else:
                self.loginfo("    Done")

    def _get_start_client_app_service(self, gatewayname):
        srv_name = '/' + gatewayname + self.postfix_start_app
        rospy.wait_for_service(srv_name)

        srv = rospy.ServiceProxy(srv_name, rapp_mamanager_srvs.StartApp)

        return srv

    def _create_startapp_request(self, app, node_name, linkgraph, service_name):

        req = rapp_mamanager_srvs.StartAppRequest()
        req.name = app

        # Remappings
        #namespace_prefix = '/' + str(service_name)
        edges = linkgraph.edges
        req.remappings = [rocon_std_msg.Remapping(e.remap_from, e.remap_to) for e in edges if e.start == node_name or e.finish == node_name]

        """
        Trial of local namespacing to guarantee service disjointness

        req.remappings = []
        for e in edges:
            fr = ''
            to = ''
            if e.start != node_name:
                fr = e.remap_from

                if e.remap_to.startswith('/'):
                    to = namespace_prefix + e.remap_to
                else:
                    to = namespace_prefix + '/' + e.remap_to
            else:
                to = e.remap_to

                if e.remap_from.startswith('/'):
                    fr = namespace_prefix + e.remap_from
                else:
                    fr = namespace_prefix + '/' + e.remap_from

            req.remappings.append(rocon_std_msg.Remapping(fr,to))
        """

        return req

    def process_resource_status(self, req):
        resp = concert_srv.ResourceStatusResponse()

        self.lock.acquire()

        available = [copy.deepcopy(self.clients[c]) for c in self.clients if not self.clients[c].gateway_name in self.inuse_clients]
        inuse = [copy.deepcopy(self.clients[c]) for c in self.clients if self.clients[c].gateway_name in self.inuse_clients]

        for a in available:
            a.apps = []
        for i in inuse:
            i.apps = []

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

            for pairs in self.pairs[s]:
                nodes, client, gateway_name = pairs
                os, version, system, platform, app, node = nodes

                p = node + " - " + client + " - " + app

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
