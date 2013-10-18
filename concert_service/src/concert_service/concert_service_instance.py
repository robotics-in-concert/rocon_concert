#!/usr/bin/env python

import rospy
from concert_msgs.msg import ConcertService, ConcertClients

class ConcertServiceInstance(object):

    description = None

    sub = {}


    def __init__(self, service_description):
        '''
            service_desciption : ConcertService.msg
        '''
        # Setting description(msg)
        self.description = service_description

        self.setup_ros_api()

        # Enable/Disable
        self.enable(self.description.enabled)


    def setup_ros_api(self):
        # Subscriber 
        self.sub['list_concert_clients'] = rospy.Subscriber('list_concert_clients',ConcertClients,self.process_list_concert_clients)

    def process_list_concert_clients(self, msg):
        self.log("Bling Bling")
        clients = msg.clients

        self.description.status = ConcertService.NOT_READY
        # TODO

    def enable(self,flag):
        self.description.enabled = flag
        # TODO : Need to be implemented

    def to_msg(self):
        return self.description
        
    def log(self,msg):
        rospy.loginfo("Service["+str(self.description.name)+"] : " + str(msg))


    def spin(self):
        self.log("In spin")
        rospy.spin()
