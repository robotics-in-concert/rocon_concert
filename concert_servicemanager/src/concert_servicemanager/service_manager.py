#!/usr/bin/env python

import rospy
import traceback
import yaml
import concert_service

from concert_msgs.msg import *
from concert_msgs.srv import *

class ServiceManager(object):

    concert_services = {}
    last_list_concert_client = []

    param = {}
    srv = {}
    pub = {}
    sub = {}

    def __init__(self):
        self.log("in init")
        self.setup_ros_api()

    def setup_ros_api(self):
        # Service
        self.srv['add_service'] = rospy.Service('service/add',AddConcertService,self.process_add_concertservice)
        self.srv['enable_service'] = rospy.Service('service/enable',EnableConcertService,self.process_enable_concertservice)

        # Publisher
        self.pub['list_service'] = rospy.Publisher('service/list',ListConcertService, latch = True)


    def process_add_concertservice(self, req):

        success = False
        reason  = "No reason"
        try:
            if req.service.name in self.concert_services:
                success = False
                reason = "Already registered service"
                self.log(reason)
            else:
                cs = concert_service.ConcertServiceInstance(req.service)
                self.concert_services[req.service.name] = cs
                self.update()
                success = True
                reason  = str(req.service.name) + " Successfully added"
        except Exception as e:
            tb = traceback.format_exc()
            success = False
            reason = "Unexpected Error while loading service"
            self.log("\n"+str(tb))

        return AddConcertServiceResponse(success,reason)

    def process_enable_concertservice(self,req):
        name = req.concertservice_name

        success = False
        reason = ""
        if req.enable:
            success, reason = self.concert_services[name].enable()
        else:
            success, reason = self.concert_services[name].disable()

        return EnableConcertServiceResponse(success,reason)

    def update(self):
        rs = [v.to_msg() for k,v in self.concert_services.items()]
        self.pub['list_service'].publish(rs)

    def log(self,msg):
        rospy.loginfo("Serivce Manager : " + str(msg))

    def spin(self):
        self.log("in spin")
        rospy.spin()
