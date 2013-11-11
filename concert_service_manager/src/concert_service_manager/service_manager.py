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
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs
import concert_roles

# Local imports
from .concert_service_instance import ConcertServiceInstance
from .service_list import load_service_descriptions_from_service_lists

##############################################################################
# ServiceManager
##############################################################################


class ServiceManager(object):

    last_list_concert_client = []
    lock = None

    def __init__(self):
        self._param = {}
        self._services = {}
        self._publishers = {}
        self._concert_services = {}
        self._setup_ros_parameters()
        self.lock = threading.Lock()
        self._role_app_loader = concert_roles.RoleAppLoader()
        roslaunch.pmon._init_signal_handlers()
        self._setup_ros_api()
        self._initialise_concert_services()

    def _initialise_concert_services(self):
        '''
          Currently only called at the end of construction.
        '''
        self.lock.acquire()
        service_descriptions = load_service_descriptions_from_service_lists(self._param['service_lists'])
        for service_description in service_descriptions:
            self._concert_services[service_description.name] = ConcertServiceInstance(service_description=service_description,
                                                                                      update_callback=self.update)
        self.lock.release()
        if self._param['auto_enable_services']:
            for service in self._concert_services.values():
                service.enable(self._role_app_loader)
        self.update()

    def _setup_ros_parameters(self):
        rospy.logdebug("Service Manager : parsing parameters")
        self._param = {}
        self._param['service_lists']        = [x for x in rospy.get_param('~service_lists', '').split(';') if x != '']  #@IgnorePep8
        self._param['auto_enable_services'] = rospy.get_param('~auto_enable_services', False)  #@IgnorePep8

    def _setup_ros_api(self):
        self._services['enable_service'] = rospy.Service('~enable', concert_srvs.EnableConcertService, self.process_enable_concertservice)
        self._publishers['list_concert_services'] = rospy.Publisher('list_concert_services', concert_msgs.ConcertServices, latch=True)

    def process_enable_concertservice(self, req):
        name = req.concertservice_name

        success = False
        message = "Not Implemented"

        if name in self._concert_services:
            if req.enable:
                success, message = self._concert_services[name].enable(self._role_app_loader)
            else:
                success, message = self._concert_services[name].disable(self._role_app_loader)
        else:
            service_names = self._concert_services.keys()
            self.loginfo("'" + str(name) + "' does not exist. Available Services = " + str(service_names))

        return concert_srvs.EnableConcertServiceResponse(success, message)

    def update(self):
        rs = [v.to_msg() for v in self._concert_services.values()]
        self._publishers['list_concert_services'].publish(rs)

    def loginfo(self, msg):
        rospy.loginfo("Service Manager : " + str(msg))

    def spin(self):
        rospy.spin()
