#!/usr/bin/env python

import rospy
import traceback
import yaml

from roconservice_instance import *
from concert_msgs.msg import *
from concert_msgs.srv import *

class ServiceManager(object):

    rocon_services = {}

    param = {}
    srv = {}
    pub = {}
    sub = {}

    def __init__(self):
        self.log("in init")

        self.setup_ros_channel()

    def setup_ros_channel(self):

        # Service
        self.srv['add_service'] = rospy.Service('service/add',AddConcertService,self.process_add_concertservice)
        self.srv['enable_service'] = rospy.Service('service/enable',EnableConcertService,self.process_enable_concertservice)

        # Publisher
        self.pub['list_service'] = rospy.Publisher('service/list',ListConcertService, latch = True)
        
        # Subscriber 
        self.sub['list_concert_clients'] = rospy.Subscriber('list_concert_clients',ConcertClients,self.process_list_concert_clients)
        

    def process_add_concertservice(self, req):
        self.log(str(req))
        self.update()

        return AddConcertServiceResponse()

    def process_enable_concertservice(self,req):

        return EnableConcertServiceResponse()

    def process_list_concert_clients(self,msg):
        '''
            Receives a list of clients
            Creates a list of tuples
            pass the list to check the current client lists fulfills services' requirements
            then pdate status of services
        '''
        self.log("Hola in list concert_clients")

    def update(self):
        rs = [v.to_msg() for k,v in self.rocon_services.items()]
        self.pub['list_service'].publish(rs)

    def log(self,msg):
        rospy.loginfo("Serivce Manger : " + str(msg))

    def spin(self):
        self.log("in spin")
        rospy.spin()
