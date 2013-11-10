#!/usr/bin/env python
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import traceback
import threading
import roslaunch.pmon
import concert_msgs.msg as concert_msg
import concert_msgs.srv as concert_srv

# Local imports
from .concert_service_instance import ConcertServiceInstance


##############################################################################
# ServiceManager
##############################################################################


class ServiceManager(object):

    concert_services = {}
    last_list_concert_client = []
    lock = None

    param = {}
    srv = {}
    pub = {}
    sub = {}

    def __init__(self):
        self.setup_ros_api()
        self.lock = threading.Lock()

        roslaunch.pmon._init_signal_handlers()

    def setup_ros_api(self):
        # Service
        self.srv['add_service'] = rospy.Service('service/add', concert_srv.AddConcertService, self.process_add_concertservice)
        self.srv['remove_service'] = rospy.Service('service/remove', concert_srv.RemoveConcertService, self.process_remove_concertservice)
        self.srv['enable_service'] = rospy.Service('service/enable', concert_srv.EnableConcertService, self.process_enable_concertservice)

        # Publisher
        self.pub['list_service'] = rospy.Publisher('list_concert_services', concert_msg.ListConcertService, latch=True)

    def process_add_concertservice(self, req):
        success = False
        message = "No reason"
        self.lock.acquire()
        try:
            if req.service.name in self.concert_services:
                success = False
                message = "Already registered service"
            else:
                cs = ConcertServiceInstance(service_description=req.service, update_callback=self.update)
                self.concert_services[req.service.name] = cs
                self.update()
                success = True
                message = str(req.service.name) + " Successfully added"
        except Exception as unused_e:
            tb = traceback.format_exc()
            success = False
            message = "Unexpected Error while loading service"
            self.loginfo("\n" + str(tb))
        self.lock.release()

        self.loginfo(message)

        return concert_srv.AddConcertServiceResponse(success, message)

    def process_remove_concertservice(self, req):
        success = False
        message = "NOT Implemented"
        self.lock.acquire()
        try:
            service_name = req.service_name
            if service_name in self.concert_services:
                if self.concert_services[service_name].is_enabled():
                    message = "Service[" + str(service_name) + "] is enabled. It should be disabled first to remove"
                    raise(Exception(message))
                else:
                    del self.concert_services[service_name]
                    success = True
                    message = str(service_name) + " Successfully removed"
                    self.update()
            else:
                message = "Service[" + str(service_name) + "] does not exist."
                raise(Exception(message))
        except Exception as e:
            success = False
            message = str(e)
        self.lock.release()

        return concert_srv.RemoveConcertServiceResponse(success, message)

    def process_enable_concertservice(self, req):
        name = req.concertservice_name

        success = False
        message = "Not Implemented"

        if name in self.concert_services:
            if req.enable:
                success, message = self.concert_services[name].enable()
            else:
                success, message = self.concert_services[name].disable()
        else:
            service_names = self.concert_services.keys()
            self.loginfo("[" + str(name) + "] does not exist. Available Services = " + str(service_names))

        return concert_srv.EnableConcertServiceResponse(success, message)

    def update(self):
        rs = [v.to_msg() for unused_k, v in self.concert_services.items()]
        self.pub['list_service'].publish(rs)

    def loginfo(self, msg):
        rospy.loginfo("Serivce Manager : " + str(msg))

    def spin(self):
        rospy.spin()
