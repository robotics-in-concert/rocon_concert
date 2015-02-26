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
#from .service_cache_manager import ServiceCacheManager
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
        '_enabled_services',     # enabled services { resource_name : ConcertServiceInstance }
        '_interactions_loader',  # rocon_interactions.InteractionLoader
        'lock',
        '_service_pool',  # manage services profile
    ]

    def __init__(self):
        """
          Initialise everything, and enable services if auto-enable is flagged.

          :raises: :exc:`rospkg.ResourceNotFound` if some resource yaml (soln configuration or service) cannot be found.
          :raises: :exc:`concert_service_manager.InvalidSolutionConfigurationException` if the service solution yaml configuration is invalid
        """
        self._enabled_services = {}
        self._parameters = self._setup_ros_parameters()
        self.lock = threading.Lock()
        self._interactions_loader = rocon_interactions.InteractionsLoader()
        roslaunch.pmon._init_signal_handlers()
        try:
            self._service_pool = ServicePool(self._parameters['concert_name'],
                                             self._parameters['solution_configuration'],
                                             self._parameters['disable_cache'],
                                             self.publish_update)

        except (rospkg.ResourceNotFound, InvalidSolutionConfigurationException) as e:
            raise e
        self._publishers = self._setup_ros_publishers()

        # auto enable service
        # default enable service
        if self._parameters['disable_cache']:
            self._eable_default_service()
        else:
            self._eable_cached_service()

        # now we let the service threads compete
        self._services = self._setup_ros_services()

    def _eable_cached_service(self):
        cached_solution_config = self._service_pool.get_solution_config()
        for cached_service in cached_solution_config.values():
            name = cached_service['name']
            enabled = cached_service['enabled']
            if name in self._service_pool.service_profiles.keys():
                if enabled is True:
                    self._ros_service_enable_concert_service(concert_srvs.EnableServiceRequest(name, True))
                elif enabled is None:
                    if self._parameters['default_auto_enable_services'] == 'all':
                        self._ros_service_enable_concert_service(concert_srvs.EnableServiceRequest(name, True))
                    elif type(self._parameters['default_auto_enable_services']) is list and name in self._parameters['default_auto_enable_services']:
                        self._ros_service_enable_concert_service(concert_srvs.EnableServiceRequest(name, True))
            else:
                rospy.logwarn("Service Manager : '%s' is not available. cannot auto enable" % str(name))

    def _eable_default_service(self):
        if self._parameters['default_auto_enable_services'] == 'all':
            for name in self._service_pool.service_profiles.keys():
                self._ros_service_enable_concert_service(concert_srvs.EnableServiceRequest(name, True))
        elif type(self._parameters['default_auto_enable_services']) is list:
            for name in self._parameters['default_auto_enable_services']:
                if name in self._service_pool.service_profiles.keys():
                    self._ros_service_enable_concert_service(concert_srvs.EnableServiceRequest(name, True))
                else:
                    rospy.logwarn("Service Manager : '%s' is not available. cannot auto enable" % str(name))
        else:
            self.publish_update()  # publish the available list

    def _setup_ros_parameters(self):
        rospy.logdebug("Service Manager : parsing parameters")
        parameters = {}
        parameters['disable_cache'] = rospy.get_param('~disable_cache', "false")
        parameters['concert_name'] = rospy.get_param('~concert_name', "")
        parameters['solution_configuration'] = rospy.get_param('~services', "")  # @IgnorePep8
        parameters['default_auto_enable_services'] = rospy.get_param('~default_auto_enable_services', [])  # @IgnorePep8
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
        services['update_service_config'] = rospy.Service('~update_service_config', concert_srvs.UpdateServiceConfig, self._ros_service_update_service_config)
        return services

    def _setup_ros_publishers(self):
        publishers = {}
        publishers['list_concert_services'] = rospy.Publisher('~list', concert_msgs.Services, latch=True, queue_size=1)
        return publishers

    def _ros_service_update_service_config(self, req):
        success = False
        message = ""
        service_profile = req.service_profile
        service_name = service_profile.name
        # write at cache
        if service_name in self._enabled_services.keys():
            success = False
            message = "%s service is running. First, stop %s service" % (service_name, service_name)
        else:
            self.lock.acquire()
            (success, message) = self._service_pool.update_service_cache(service_profile)
            self.lock.release()

        return concert_srvs.UpdateServiceConfigResponse(success, message)

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
                self._service_pool.reload_services()
                # Check if the service name is in the currently loaded service profiles
                if name not in self._enabled_services.keys():
                    try:
                        service_instance = ServiceInstance(self._parameters['concert_name'], self._parameters['disable_cache'], self._service_pool.find(name).msg)
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
        self._service_pool.update_solution_configuration(services)
        self._publishers['list_concert_services'].publish(services)

    def loginfo(self, msg):
        rospy.loginfo("Service Manager : " + str(msg))

    def logwarn(self, msg):
        rospy.logwarn("Service Manager : " + str(msg))

    def spin(self):
        while not rospy.is_shutdown():
            self.lock.acquire()
            self._service_pool.reload_services()
            self.lock.release()
            rospy.sleep(0.5)
