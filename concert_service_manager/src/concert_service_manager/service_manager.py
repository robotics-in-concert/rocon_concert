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
import rocon_python_utils
import rocon_std_msgs.msg as rocon_std_msgs

from .exceptions import NoServiceExistsException
from concert_service_manager.service_instance import ServiceInstance
from .service_profiles import load_service_profile
#from concert_service_manager.service_pool import SolutionConfiguration
from .exceptions import InvalidSolutionConfigurationException, InvalidServiceProfileException

##############################################################################
# ServiceManager
##############################################################################


class ServiceManager(object):

    __slots__ = [
            '_parameters',
            '_services',
            '_publishers',
            '_concert_services',        # { resource_name : ConcertServiceInstance }
            '_interactions_loader',     # rocon_interactions.InteractionLoader
            '_solution_configuration',  # holds the solution's service related configuration data : .solution_configuration.SolutionConfiguration
            'lock'
        ]

    def __init__(self):
        """
          Initialise everything, and enable services if auto-enable is flagged.

          :raises: :exc:`rospkg.ResourceNotFound` if some resource yaml (soln configuration or service) cannot be found.
          :raises: :exc:`concert_service_manager.InvalidSolutionConfigurationException` if the service solution yaml configuraiton is invalid
        """
        self._concert_services = {}
        self._parameters = self._setup_ros_parameters()
        self.lock = threading.Lock()
        self._interactions_loader = rocon_interactions.InteractionsLoader()
        roslaunch.pmon._init_signal_handlers()
        try:
            self._solution_configuration = SolutionConfiguration(self._parameters['solution_configuration'])
        except (rospkg.ResourceNotFound, InvalidSolutionConfigurationException) as e:
            raise e
        self._initialise_concert_services()
        (self._services, self._publishers) = self._setup_ros_api()
        # do we need a small sleep here to let the ros api construct?
        self.publish_update()

    def _initialise_concert_services(self):
        """
          Don't need to lock here as we don't introduce the ros api callbacks to compete with this yet.

          :raises: :exc:`rospkg.ResourceNotFound` if service profile cannot be found.
          :raises: :exc:`concert_service_manager.InvalidServiceProfileException` if service profile is invalid
        """
        service_profiles = []  # [ name : concert_msgs.ServiceProfile ]
        for service_data in self._solution_configuration.services:
            try:
                filename = self._known_concert_services[service_data.resource_name]
            except KeyError:
                raise rospkg.ResourceNotFound("could not find the service profile [%s]" % service_data.resource_name)
            try:
                service_profile = load_service_profile(service_data.resource_name, service_data.overrides, filename)  # concert_msgs.ServiceProfile
                service_profiles.append(service_profile)
            except InvalidServiceProfileException as e:
                raise e
        for service_profile in service_profiles:
            if service_profile.name in self._concert_services.keys():
                # could be a bit more meaningful, i.e. show the resource names for both original and found duplicate
                rospy.logerr("Service Manager : duplicate service profile name discovered, not loading [%s]" % service_profile.name)
                continue
            self._concert_services[service_profile.name] = ServiceInstance(service_profile=service_profile, update_callback=self.publish_update)

        if self._parameters['auto_enable_services']:
            for name in self._concert_services.keys():
                self._ros_service_enable_concert_service(concert_srvs.EnableServiceRequest(name, True))

    def _setup_ros_parameters(self):
        rospy.logdebug("Service Manager : parsing parameters")
        parameters = {}
        parameters['solution_configuration'] = rospy.get_param('~services', [])  #@IgnorePep8
        parameters['auto_enable_services']   = rospy.get_param('~auto_enable_services', False)  #@IgnorePep8
        return parameters

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
        services = {}
        services['enable_service'] = rospy.Service('~enable', concert_srvs.EnableService, self._ros_service_enable_concert_service)
        publishers = {}
        publishers['list_concert_services'] = rospy.Publisher('~list', concert_msgs.Services, latch=True)
        return (services, publishers)

    def _ros_service_enable_concert_service(self, req):

        name = req.name
        success = False
        message = "unknown error"

        if req.enable:
            self.loginfo("serving request to enable '%s'" % name)
        else:
            self.loginfo("serving request to disable '%s'" % name)

        self.lock.acquire()  # could be an expensive lock?

        # Reload solution configuration and provide simple checks/warnings if a problem is found..
        self._solution_configuration.reload()
        undiscoverable_services = [s.resource_name for s in self._solution_configuration.services if s.resource_name not in self._known_concert_services.keys()]
        if undiscoverable_services:
            self._known_concert_services, unused_invalid_services = rocon_python_utils.ros.resource_index_from_package_exports(rocon_std_msgs.Strings.TAG_SERVICE)
            undiscoverable_services = [s.resource_name for s in self._solution_configuration.services if s.resource_name not in self._known_concert_services.keys()]
            rospy.logwarn("Service Manager : services were configured, but could not be found on the package path %s" % undiscoverable_services)

        try:
            if req.enable:
                # Check if the service name is in the currently loaded service profiles
                if name not in self._concert_services.keys():
                    service_data = self._solution_configuration.find(name)
                    if service_data is None:
                        service_profile = None
                        possible_service_data_list = self._solution_configuration.unnamed()
                        for s in possible_service_data_list:
                            try:
                                filename = self._known_concert_services[s.resource_name]
                            except KeyError:
                                # we already know from earlier check that it is undiscoverable and issued warnings
                                # just skip past it here
                                continue
                            profile = load_service_profile(s.resource_name, s.overrides, filename)
                            if profile.name == name:
                                service_profile = profile
                                break
                    else:
                        service_profile = load_service_profile(service_data.resource_name, service_data.overrides, filename)
                    if service_profile is None:
                        raise NoServiceExistsException("service not found on the package path [%s]" % name)
                    self._concert_services[service_profile.name] = ServiceInstance(service_profile=service_profile, update_callback=self.publish_update)
                else:
                    # TODO : check that we are still actually permitted to launch it (i.e. look in solution_configuration service data
                    pass

                unique_identifier = unique_id.fromRandom()
                self._setup_service_parameters(self._concert_services[name].profile.name,
                                               self._concert_services[name].profile.description,
                                               unique_identifier)
                success, message = self._concert_services[name].enable(unique_identifier, self._interactions_loader)
                if not success:
                    self._cleanup_service_parameters(self._concert_services[name].profile.name)
            else:
                if not name in self._concert_services:
                    raise NoServiceExistsException("no such service in the concert [%s]" % name)

                # Cause an error if it tries to disable a service that is already disabled.
                if self._concert_services[name].is_enabled():
                    self._cleanup_service_parameters(self._concert_services[name].profile.name)
                success, message = self._concert_services[name].disable(self._interactions_loader, self._unload_resources)
                # todo delete the service instance if we check solution configuration and cant find a match
        except NoServiceExistsException as e:
            rospy.logwarn("Service Manager : %s" % str(e))
            success = False

        self.lock.release()
        self.publish_update()
        return concert_srvs.EnableServiceResponse(success, message)

#     def _reload_solution_configuration(self):
#         '''
#             Load service profiles from solution file and update disabled services configuration
#         '''
#         try:
#             service_profiles = load_service_profiles(self._parameters['services'])
# 
#             # replace configurations of disabled services
# 
#             # Update configuration of disabled services
#             deleted_services = {s: v for s, v in self._concert_services.items() if not v.is_enabled() and not s in service_profiles}
#             for s in deleted_services:
#                 del self._concert_services[s]
# 
#             disabled_services = {s: service_profiles[s] for s, v in self._concert_services.items() if not v.is_enabled() and s in service_profiles}
#             self._load_services(disabled_services)
# 
#             # Load newly added services
#             new_services = {s: v for s, v in service_profiles.items() if not s in self._concert_services}
#             self._load_services(new_services)
# 
#         except NoConfigurationUpdateException as unused_e:
#             # It is just escaping mechanism if there is nothing to update in service configuration
#             pass

    def publish_update(self):
        rs = [v.to_msg() for v in self._concert_services.values()]
        self._publishers['list_concert_services'].publish(rs)

    def loginfo(self, msg):
        rospy.loginfo("Service Manager : " + str(msg))

    def logwarn(self, msg):
        rospy.logwarn("Service Manager : " + str(msg))

    def spin(self):
        rospy.spin()
