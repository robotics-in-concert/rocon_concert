# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import threading

import roslaunch.pmon
import rospkg
import rospy
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs
import rocon_interactions
import unique_id

from .exceptions import NoServiceExistsException
from .service_instance import ServiceInstance
from .service_pool import ServicePool
from .exceptions import InvalidSolutionConfigurationException

##############################################################################
# ServiceManager
##############################################################################


class ServiceManager(object):

    __slots__ = [
            '_parameters',
            '_services',
            '_publishers',
            '_service_pool',         # all service profiles that are permitted to be enabled, ServicePool
            '_enabled_services',     # enabled services { resource_name : ConcertServiceInstance }
            '_interactions_loader',  # rocon_interactions.InteractionLoader
            'lock'
        ]

    def __init__(self):
        """
          Initialise everything, and enable services if auto-enable is flagged.

          :raises: :exc:`rospkg.ResourceNotFound` if some resource yaml (soln configuration or service) cannot be found.
          :raises: :exc:`concert_service_manager.InvalidSolutionConfigurationException` if the service solution yaml configuraiton is invalid
        """
        self._enabled_services = {}
        self._parameters = self._setup_ros_parameters()
        self.lock = threading.Lock()
        self._interactions_loader = rocon_interactions.InteractionsLoader()
        roslaunch.pmon._init_signal_handlers()
        try:
            self._service_pool = ServicePool(self._parameters['solution_configuration'])
        except (rospkg.ResourceNotFound, InvalidSolutionConfigurationException) as e:
            raise e
        self._publishers = self._setup_ros_publishers()
        if self._parameters['auto_enable_services']:
            for name in self._service_pool.service_profiles.keys():
                self._ros_service_enable_concert_service(concert_srvs.EnableServiceRequest(name, True))
        else:
            self.publish_update()  # publish the available list
        # now we let the service threads compete
        self._services = self._setup_ros_services()

    def _setup_ros_parameters(self):
        rospy.logdebug("Service Manager : parsing parameters")
        parameters = {}
        parameters['solution_configuration'] = rospy.get_param('~services', "")  #@IgnorePep8
        parameters['auto_enable_services']   = rospy.get_param('~auto_enable_services', False)  #@IgnorePep8
        return parameters

    def _setup_service_parameters(self, name, description, priority, unique_identifier):
        '''
          Dump some important information for the services to self-introspect on in the namespace in which
          they will be started.

          :param str name: text name for the service (unique)
          :param str description: text description of the service
          :param int priority: a numeric priority level that can be configured at service level for establishing resource requests
          :param uuid.UUID unique_identifier: unique id for the service
        '''
        namespace = concert_msgs.Strings.SERVICE_NAMESPACE + '/' + name
        rospy.set_param(namespace + "/name", name)
        rospy.set_param(namespace + "/description", description)
        rospy.set_param(namespace + "/priority", priority)
        rospy.set_param(namespace + "/uuid", unique_id.toHexString(unique_id.toMsg(unique_identifier)))

    def _cleanup_service_parameters(self, name):
        namespace = concert_msgs.Strings.SERVICE_NAMESPACE + '/' + name
        rospy.delete_param(namespace + "/name")
        rospy.delete_param(namespace + "/description")
        rospy.delete_param(namespace + "/priority")
        rospy.delete_param(namespace + "/uuid")

    def _setup_ros_services(self):
        services = {}
        services['enable_service'] = rospy.Service('~enable', concert_srvs.EnableService, self._ros_service_enable_concert_service)
        return services

    def _setup_ros_publishers(self):
        publishers = {}
        publishers['list_concert_services'] = rospy.Publisher('~list', concert_msgs.Services, latch=True)
        return publishers

    def _ros_service_enable_concert_service(self, req):

        name = req.name
        success = False
        message = "unknown error"

        if req.enable:
            self.loginfo("serving request to enable '%s'" % name)
        else:
            self.loginfo("serving request to disable '%s'" % name)

        self.lock.acquire()  # could be an expensive lock?

        # DJS : reload the service pool
        try:
            if req.enable:
                self._service_pool.reload()  # check if the solution specs have updated
                # Check if the service name is in the currently loaded service profiles
                if name not in self._enabled_services.keys():
                    try:
                        service_instance = ServiceInstance(self._service_pool.find(name).msg, update_callback=self.publish_update)
                    except NoServiceExistsException:
                        # do some updating of the service pool here
                        raise NoServiceExistsException("service not found on the package path [%s]" % name)
                    unique_identifier = unique_id.fromRandom()
                    self._setup_service_parameters(service_instance.msg.name,
                                                   service_instance.msg.description,
                                                   service_instance.msg.priority,
                                                   unique_identifier)
                    success, message = service_instance.enable(unique_identifier, self._interactions_loader)
                    if not success:
                        self._cleanup_service_parameters(service_instance.msg.name)
                    else:
                        self._enabled_services[service_instance.name] = service_instance
                else:
                    success = True
                    message = "already enabled"
            else:
                if not name in self._enabled_services:
                    raise NoServiceExistsException("no enabled service with that name [%s]" % name)
                self._cleanup_service_parameters(self._enabled_services[name].msg.name)
                success, message = self._enabled_services[name].disable(self._interactions_loader)
                del self._enabled_services[name]
        except NoServiceExistsException as e:
            rospy.logwarn("Service Manager : %s" % str(e))
            success = False
            message = str(e)
        self.publish_update()
        self.lock.release()
        return concert_srvs.EnableServiceResponse(success, message)

    def publish_update(self):
        '''
          This is not locked here - it should always be called inside a locked scope.
        '''
        services = [service_profile.msg for service_profile in self._service_pool.service_profiles.values()]
        for service in services:
            service.enabled = True if service.name in self._enabled_services.keys() else False
        self._publishers['list_concert_services'].publish(services)

    def loginfo(self, msg):
        rospy.loginfo("Service Manager : " + str(msg))

    def logwarn(self, msg):
        rospy.logwarn("Service Manager : " + str(msg))

    def spin(self):
        rospy.spin()
