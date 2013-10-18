#!/usr/bin/env python

import rospy
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

        #  Service
#        self.srv['enable_service'] = rospy.Service('service/enable',EnableConcertService,self.process_enable_concertservice)

    def process_list_concert_clients(self, msg):
        self.log("Bling Bling")

        self.lock.acquire()
        self.list_concert_client = msg.clients

        # Validate the table 
        clients = msg.clients
        client_list = [(c.name, str(c.platform) + "." +  str(c.system) + "." +str(c.robot)) for c in clients]
        self.log("\n" + str(client_list))
        self.description.status = self.compatibility_table.is_ready(client_list)
        self.lock.release()


    def process_enable_concertservice(self,req):

        success = False
        reason = ""
        if req.enable:
            success, reason = self.enable()
        else:
            success, reason = self.disable()

        return EnableConcertServiceResponse(success,reason)

    def enable(self):

        self.lock.acquire()
        # if status is valid,  Starts up clients' app
        # flag service as enabled
        # if not, return false and reason
        self.lock.release()

        return False, "Not Implemented"

    def disable(self):
        self.lock.acquire()
        # Take down all clients.
        # update the status
        # Flag service as disabled
        self.lock.release()

        return False, "Not Implemented"


    def to_msg(self):
        return self.description
        
    def log(self,msg):
        rospy.loginfo("Service["+str(self.description.name)+"] : " + str(msg))


    def spin(self):
        self.log("In spin")
        rospy.spin()
