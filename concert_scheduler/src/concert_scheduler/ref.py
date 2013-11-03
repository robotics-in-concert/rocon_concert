#!/usr/bin/env python

import rospy
import traceback
import threading
import concert_msgs.srv as concert_srv
import concert_msgs.msg as concert_msg
import rocon_std_msgs.msg as rocon_std_msg
import rocon_app_manager_msgs.srv as rapp_mamanager_srvs

from .compatibility_tree import *
from .exceptions import *


class ConcertSchedular(object):

    description = None
    list_concert_clients = []
    lock = None # To prevent updating client list and validation while enabling service 

    sub = {}
    srv = {}

    postfix_start_app = '/start_app'
    postfix_stop_app = '/stop_app'

    app_pairs = []
    current_pairs = []
    message = ""


    def __init__(self, service_description):
        '''
            service_desciption : ConcertService.msg
        '''
        self.lock = threading.Lock()
        # Setting description(msg)
        self.description = service_description

        self.init_compatibility_tree(self.description.linkgraph)
        self.setup_ros_api()

        # Enable/Disable
#        self.enable(self.description.enabled)

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        self.log("Destroying...")
        self.log("Disabling... service")
        self.disable()
        self.log("Shutting down subscriber")
# self.sub['list_concert_clients'].unregister()
        self.log("Destroying...done")

    def init_compatibility_tree(self,linkgraph):
        self.compatibility_tree = CompatibilityTree(linkgraph.dedicated_nodes)

    def is_enabled(self):
        return self.description.enabled


    def setup_ros_api(self):
        pass
        # Subscriber 
#        self.sub['list_concert_clients'] = rospy.Subscriber('list_concert_clients',concert_msg.ConcertClients,self.process_list_concert_clients)
        self.srv['lock_concert_clients'] = rospy.ServiceProxy('conductor/lock_concert_clients',concert_srv.LockConcertClients)


    def process_list_concert_clients(self, msg):

        '''
            - Check if all of dedicated clients are still alive. Mark as disabled if dedicated client has left.
            - Maintain the status of service whether it is ready, not ready
        '''
        self.log("Bling Bling")
        self.lock.acquire()
        self.list_concert_clients = msg.clients

        if self.description.enabled:
            if self.is_still_valid(self.list_concert_clients): # enabled and valid. do nothing
                self.log("Still valid service")
                self.lock.release()
                return
            else:
                self._disable() # if the current pair is not valid anymore, take the service down
                self.log("Service is not valid anymore. Disabled")

        self.refresh_app_client_pairs()

        self.lock.release()

    def refresh_app_client_pairs(self):
        # update the pair with new client list
        client_list = self.generate_available_client_list(self.list_concert_clients)
        self.description.status, self.message, self.app_pairs = self.compatibility_tree.is_ready(client_list)
        self.log("Status : " + str(self.description.status))
        self.log("message : " + str(self.message))
        self.print_pairs(self.app_pairs)


    def is_still_valid(self,list_client):
        client_names = set([ c.gateway_name for c in list_client])
        pair_names = set([ gatewayname for nodes, client, gatewayname in self.current_pairs])

        result = pair_names.issubset(client_names)

        if not result:
            diff = pair_names.difference(client_names) 
            self.log("Left clients : " + str(diff))

        return pair_names.issubset(client_names)
            

    def generate_available_client_list(self, clients):
        clients = [c for c in clients if c.client_status == concert_msg.Constants.CONCERT_CLIENT_STATUS_CONNECTED and c.app_status == concert_msg.Constants.APP_STATUS_STOPPED]

        return clients

    def enable(self):
        self.log("In Enable")
        self.lock.acquire()
        success, message = self._enable()
        self.lock.release()

        r = "Success" if success else "Failed"
        self.log(r)

        return success, message 

    def _enable(self):
        if self.description.status != concert_msg.ConcertService.READY:
            return False, self.message 

        success = False
        message  = "NOT IMPLEMENTED"

        try:
            self.current_pairs = self.app_pairs
            # TODO Request conductor to lock clients
            success, message = self.request_to_lock_clients(self.current_pairs)

            if not success:
                m = "Failed to lock clients. Please try again ["+str(message)+"]"
                # TODO : If failed to lock, should not call _disable()
                raise FailedToLockClientsException(m)
            
            # TODO Starts up apps if success to lock all required 
            self.start_apps(self.current_pairs)
            self.description.enabled = True
            success = True
            message = "Successfully apps are started"
        except FailedToLockClientsException as e:
            success = False
            message = str(e)
            self.log(str(e))
        except FailedToStartAppsException as e:
            success = False
            message = str(e)
            self.log(str(e))

            # To stop apps which already started
            self._disable()
        
        return success, message

    def request_to_lock_clients(self,pairs):
        client_names = [ client for nodes, client, client_gatewayname in pairs]

        req = concert_srv.LockConcertClientsRequest(client_names)
        
        resp = self.srv['lock_concert_clients'](req)
        return resp.success, resp.message


    def disable(self):
        '''
            wrapper function to lock. Disable may get called from various places.
        '''
        self.log("In disable")
        self.lock.acquire()
        success, message = self._disable()
        self.refresh_app_client_pairs()
        self.lock.release()

        return success, message 

    def _disable(self):
        '''
        # Take down all clients.
        # update the status
        # Request conductor to unlock clients
        # Flag service as disabled

        What if app is running which started by others?...
        '''

        success = False
        message  = "NOT IMPLEMENTED"

        try:
            # ignore clients which already left. Take down clients which are in list concert client
            pairs = self.get_remaining_pairs()
            self.stop_apps(pairs)
            success = True
            message  = "Successfully stopped all clients" 
        except Exception as e:
            success = False
            message = e
            self.log(str(e))
            pass

        self.description.enabled = False

        return success, message

    def stop_apps(self,pairs):
        for nodes, client, client_gatewayname in pairs:
            _, __, ___, app, node = nodes
            self.log("Node : " + str(node) + "\tApp : " + str(app) + "\tClient : " + str(client))

            # Creates stop app service
            stop_app_srv = self.get_stop_client_app_service(client_gatewayname)

            # Create stop app request object
            req = rapp_mamanager_srvs.StopAppRequest()
            
            # Request
            self.log("    Stopping...")
            resp = stop_app_srv(req)

            if not resp.stopped:
                message = "Failed to stop[" + str(app) + "] in [" + str(client) +"]"
                raise Exception(message)   
            else:                                 
                self.log("    Done")

    def get_remaining_pairs(self):
        list_client = self.list_concert_clients
        client_names = [ c.gateway_name for c in list_client]
        pairs = [ (nodes, client, gatewayname) for nodes, client, gatewayname in self.current_pairs if gatewayname in client_names]

        return pairs


    def start_apps(self,pairs):
        for nodes, client, client_gatewayname in pairs:
            _, __, ___, app, node = nodes
            self.log("Node : " + str(node) + "\tApp : " + str(app) + "\tClient : " + str(client))

            # Creates start app service
            start_app_srv = self.get_start_client_app_service(client_gatewayname)

            # Create start app request object
            req = self.create_startapp_request(app,node)

            # Request client to start app 
            self.log("    Starting...")
            self.log(str(req.remappings))
            resp = start_app_srv(req)

            if not resp.started:
#                message = "Failed to start [" + str(app) + "] in [" + str(client) +"]"
                message = resp.message
                raise FailedToStartAppsException(message)
            else:
                self.log("    Done")
                

    
    def get_start_client_app_service(self,gatewayname):
        srv_name = '/' + gatewayname + self.postfix_start_app
        rospy.wait_for_service(srv_name)

        srv = rospy.ServiceProxy(srv_name, rapp_mamanager_srvs.StartApp)

        return srv

    def get_stop_client_app_service(self,gatewayname):
        srv_name = '/' + gatewayname + self.postfix_stop_app
        rospy.wait_for_service(srv_name)

        srv = rospy.ServiceProxy(srv_name, rapp_mamanager_srvs.StopApp)

        return srv

    def create_startapp_request(self, app, node_name):

        req = rapp_mamanager_srvs.StartAppRequest()
        req.name = app

        # Remappings
        edges = self.description.linkgraph.edges
        req.remappings = [ rocon_std_msg.Remapping(e.remap_from, e.remap_to) for e in edges if e.start == node_name or e.finish == node_name ]

        return req

    def to_msg(self):
        return self.description
        
    def log(self,msg):
        rospy.loginfo("Service["+str(self.description.name)+"] : " + str(msg))

    def print_pairs(self,pair):
        self.log("App Pairs")
        for nodes, client, gatewayname in pair:
            _, __, ___, app, node = nodes
            self.log("Node : " + str(node) + "\tApp : " + str(app) + "\tClient : " + str(client))

    def spin(self):
        self.log("In spin")
        rospy.spin()
