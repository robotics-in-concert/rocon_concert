# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import threading
import roslaunch.pmon
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs
import rocon_interactions
import unique_id

from .exceptions import NoConfigurationUpdatException
from .concert_service_instance import ConcertServiceInstance
from .service_profiles import load_service_profiles

##############################################################################
# ServiceManager
##############################################################################


class ServiceManager(object):

    def __init__(self):
        self._param = {}
        self._services = {}
        self._publishers = {}
        self._concert_services = {}
        self._setup_ros_parameters()
        self.lock = threading.Lock()
        self._interactions_loader = rocon_interactions.InteractionsLoader()
        roslaunch.pmon._init_signal_handlers()
        self._setup_ros_api()
        self._initialise_concert_services()

    def _initialise_concert_services(self):
        '''
          Currently only called at the end of service manager construction.
        '''
        service_profiles = load_service_profiles(self._param['services'])
        self._load_services(service_profiles)

        if self._param['auto_enable_services']:
            for resource in service_profiles.keys():
                self._ros_service_enable_concert_service(concert_srvs.EnableServiceRequest(resource, True))
        self.update()

    def _setup_ros_parameters(self):
        rospy.logdebug("Service Manager : parsing parameters")
        self._param = {}
        self._param['services']        = rospy.get_param('~services', [])  #@IgnorePep8
        self._param['auto_enable_services'] = rospy.get_param('~auto_enable_services', False)  #@IgnorePep8

    def _setup_service_parameters(self, name, description, unique_identifier):
        '''
          Dump some important information for the services to self-introspect on in the namespace in which
          they will be started.

          @param name : text name for the service (unique)
          @type str

          @param description : text description of the service
          @type str

          @param unique_identifier : unique id for the service
          @type uuid.UUID
        '''
        namespace = concert_msgs.Strings.SERVICE_NAMESPACE + '/' + name
        rospy.set_param(namespace + "/name", name)
        rospy.set_param(namespace + "/description", description)
        rospy.set_param(namespace + "/uuid", unique_id.toHexString(unique_id.toMsg(unique_identifier)))

    def _cleanup_service_parameters(self, name):
        namespace = concert_msgs.Strings.SERVICE_NAMESPACE + '/' + name
        rospy.delete_param(namespace + "/name")
        rospy.delete_param(namespace + "/description")
        rospy.delete_param(namespace + "/uuid")

    def _setup_ros_api(self):
        self._services['enable_service'] = rospy.Service('~enable', concert_srvs.EnableService, self._ros_service_enable_concert_service)
        self._publishers['list_concert_services'] = rospy.Publisher('~list', concert_msgs.ConcertServices, latch=True)

    def _unload_resources(self, service_name):
        # Taken out temporarily until the scheduler handles 'groups',
        pass
        #request_resources = concert_msgs.RequestResources()
        #request_resources.service_name = service_name
        #request_resources.enable = False
        #self._publishers['request_resources'].publish(request_resources)

    def _ros_service_enable_concert_service(self, req):
        resource = req.resource

        success = False
        message = "unknown error"

        if req.enable:
            self.loginfo("serving request to enable '%s'" % resource)
        else:
            self.loginfo("serving request to disable '%s'" % resource)
        if resource in self._concert_services:
            if req.enable:
                self._reload_solution_configuration()
                unique_identifier = unique_id.fromRandom()
                self._setup_service_parameters(self._concert_services[resource].profile.name,
                                               self._concert_services[resource].profile.description,
                                               unique_identifier)
                success, message = self._concert_services[resource].enable(unique_identifier, self._interactions_loader)
                if not success:
                    self._cleanup_service_parameters(self._concert_services[resource].profile.name)
            else:
                self._cleanup_service_parameters(self._concert_services[resource].profile.name)
                success, message = self._concert_services[resource].disable(self._interactions_loader, self._unload_resources)
                self._reload_solution_configuration()
        else:
            service_names = self._concert_services.keys()
            message = "'" + str(resource) + "' does not exist " + str(service_names)
            self.logwarn(message)
            success = False
        self.update()
        return concert_srvs.EnableServiceResponse(success, message)

    def _reload_solution_configuration(self):
        '''
            Load service profiles from solution file and update disabled services configuration
        '''
        try:
            service_profiles = load_service_profiles(self._param['services'])
            # Load newly added services
            unloaded_services = {s: v for s, v in service_profiles.items() if not s in self._concert_services}
            self._load_services(unloaded_services)
            # Update configuration of disabled services
            disabled_services = {s: service_profiles[s] for s, v in self._concert_services.items() if not v.is_enabled()}
            self._load_services(disabled_services)
            # TODO :What if service has been removed from solution configuration??
        except NoConfigurationUpdatException as e:
            # It is just escaping mechanism if there is nothing to update in service configuration
            pass

    def _load_services(self, service_profiles):
        self.lock.acquire()
        for resource, service_profile in service_profiles.items():
            self._concert_services[resource] = ConcertServiceInstance(service_profile=service_profile,
                                                                                      update_callback=self.update)
        self.lock.release()

    def update(self):
        rs = [v.to_msg() for v in self._concert_services.values()]
        self._publishers['list_concert_services'].publish(rs)

    def loginfo(self, msg):
        rospy.loginfo("Service Manager : " + str(msg))

    def logwarn(self, msg):
        rospy.logwarn("Service Manager : " + str(msg))

    def spin(self):
        rospy.spin()
