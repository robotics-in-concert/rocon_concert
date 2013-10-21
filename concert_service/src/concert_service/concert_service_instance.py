#!/usr/bin/env python

import rospy
import traceback
import threading
from .compatibility_table import *
from concert_msgs.msg import ConcertService, ConcertClients
from concert_msgs.srv import *

class ConcertServiceInstance(object):

    description = None
    list_concert_client = []
    lock = None # To prevent updating client list and validation while enabling service 

    sub = {}


    def __init__(self, service_description):
        '''
            service_desciption : ConcertService.msg
        '''
        self.lock = threading.Lock()
        # Setting description(msg)
        self.description = service_description
        self.init_compatibility_table(self.description.linkgraph)
        self.setup_ros_api()

        # Enable/Disable
#        self.enable(self.description.enabled)

    def init_compatibility_table(self,linkgraph):
        self.compatibility_table = CompatibilityTable(linkgraph.dedicated_nodes)


    def setup_ros_api(self):
        # Subscriber 
        self.sub['list_concert_clients'] = rospy.Subscriber('list_concert_clients',ConcertClients,self.process_list_concert_clients)



    def process_list_concert_clients(self, msg):
        self.log("Bling Bling")

        self.lock.acquire()
        self.list_concert_client = msg.clients

        # Validate the table 
        clients = msg.clients
        client_list = [(c.name, str(c.platform) + "." +  str(c.system) + "." +str(c.robot)) for c in clients]
        self.log("\n" + str(client_list))
        self.description.status, self.app_pairs = self.compatibility_table.is_ready(client_list)
        self.lock.release()

    def enable(self):

        self.lock.acquire()
        # if status is valid,  Starts up clients' app
        success = False
        reason = "Bad"
        if self.description.status == ConcertClients.READY:
            try:
                # Starts apps
                self.starts_apps()

                # Flag service as enabled
                success = True
                reason = "Successfully enabled"
            except Exception as e:
                # Any issue?
                tb = traceback.format_exc()
                success = False
                reason = "Unexpected Error while loading service"
                self.log("\n"+str(tb))
        else:
            reason = "Service is not ready"
            success = False
            self.description.enabled = False
        
        # flag service as enabled
        # if not, return false and reason
        self.lock.release()

        return sucess, reason 

    def disable(self):
        self.lock.acquire()
        # Take down all clients.
        # update the status
        # Flag service as disabled
        self.lock.release()

        return False, "Not Implemented"

    def starts_apps(self):
#        for app, client in self.app_pairs:



    def to_msg(self):
        return self.description
        
    def log(self,msg):
        rospy.loginfo("Service["+str(self.description.name)+"] : " + str(msg))


    def spin(self):
        self.log("In spin")
        rospy.spin()
