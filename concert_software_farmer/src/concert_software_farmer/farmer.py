# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################
import time
import rospy
import roslaunch
import concert_msgs.srv as concert_srvs
import concert_msgs.msg as concert_msgs
import threading
from .pool import SoftwarePool
from .instance import SoftwareInstance
from .exceptions import SoftwareInstanceException, InvalidSoftwareprofileException, SoftwareProfileException, SoftwareNotExistException

##############################################################################
# Softwarefarmer 
##############################################################################

class SoftwareFarmer(object):
    
    __slots__ = ['_params', '_software_pool', '_running_software', '_pub', '_srv', '_lock', '_timeout']

    def __init__(self):
        self._params = self._setup_ros_parameters()
        self._setup_ros_apis()
        self._software_pool = SoftwarePool()
        self._running_software = {}
        self._lock = threading.Lock()
        self._timeout = rospy.get_param('~release_timeout', 15) #sec


        roslaunch.pmon._init_signal_handlers()
        rospy.on_shutdown(self._wait_for_release_software)

    def _wait_for_release_software(self):
        '''
          Wait for releasing the allocating software
        '''
        timeout_time = time.time() + self._timeout
        while len(self._running_software) and time.time() < timeout_time:
            rospy.sleep(0.2)
        if len(self._running_software):
            self.logwarn('Release timeout [%s]' % str(self._running_software.keys()))

    def _setup_ros_parameters(self):
        '''
          Read ros params to configure SoftwareFarmer Instance. Currently it does not read any param yet.
        '''
        params = {}
        return params

    def _setup_ros_apis(self):
        '''
          Setup ROS publisher and service to provide farmer status and receive allocation request          
        '''
        pub = {}
        pub['list']   = rospy.Publisher('~list',concert_msgs.SoftwareProfiles,latch=True, queue_size=1)
        pub['status'] = rospy.Publisher('~status', concert_msgs.SoftwareInstances, latch=True, queue_size=1)

        srv = {}
        srv['allocate'] = rospy.Service('~allocate', concert_srvs.AllocateSoftware, self._process_allocate_software)
        
        self._pub = pub
        self._srv = srv

    def _process_allocate_software(self, req): 
        '''
          (De)allocate software based on users request
        '''
        self._lock.acquire()
        if req.allocate:
            response = self._allocate_software(req.software, req.user, req.parameters)
        else:
            response = self._deallocate_software(req.software, req.user)
        self._lock.release()
        self.loginfo("%s['%s']"%(response.success, response.error_message))
        self.pub_instance_status()
        return response

    def _allocate_software(self, software_name, user, parameters):
        '''
          Allocate software to given user
          TODO: currently it does not handle the case like an user requests software already running with different parameter configuration.
                Later we may upgrade this to manage software with different parameter set as different instances

          :param software_name str: software name to allocate
          :param user str: user who requested allocation
          :param parameters rocon_std_msgs.KeyValue[]: parameters for software
        '''
        resp = concert_srvs.AllocateSoftwareResponse()
        self.loginfo("User[%s] requested to use %s"%(user, software_name))
        if software_name in self._running_software.keys():
            instance = self._running_software[software_name]
            if instance.is_max_capacity(): 
                resp.success = False
                resp.error_message = "It exceeds software capacity"
            else:
                success, num_user = instance.add_user(user)
        
                if success:
                    resp.success = True
                    resp.namespace = instance.get_namespace()
                else:
                    resp.success = False
                    resp.error_message = "User[%s] already exist"%str(user)
        else:
            try:
                software_profile = self._software_pool.get_profile(software_name)
                instance = SoftwareInstance(software_profile, parameters)
                instance.start(user)
                self._running_software[software_name] = instance
                resp.success = True
                resp.namespace = instance.get_namespace() 
                resp.parameters = instance.get_parameters()
            except SoftwareNotExistException as e:
                resp.success= False
                resp.error_message = str(e)
            except (SoftwareProfileException, SoftwareInstanceException) as e:
                if instance:
                    instance.stop()
                resp.success = False
                resp.error_message = str(e)
        return resp

    def _deallocate_software(self, software_name, user):
        '''
          Deallocate software for given user
                                                              
          :param software_name str: software name to deallocate
          :param user str: user who requested deallocation
        '''
        self.loginfo("User[%s] requested to cancel %s"%(user, software_name))
        success = False
        message = ""
        if software_name in self._running_software.keys():
            instance = self._running_software[software_name]           
            success, num_user = instance.remove_user(user)
            if success: 
                if num_user == 0:
                    instance.stop()
                    del self._running_software[software_name]
                message = "success!"
            else:
                message = "%s not exist"%str(user)
        else:
            success = False
            message = "%s is not running!"%str(software_name)

        return concert_srvs.AllocateSoftwareResponse(success,[],"",message)

    def spin(self):
        '''
          Spinner
        '''
        self._print_pool_status()
        self.pub_pool_status()
        self.pub_instance_status()
        rospy.spin()

    def pub_instance_status(self):
        '''
          Publishes the status of instances currently running. It shows which software are running and who users are
        '''
        instances = self._running_software.values() 
        msg = [i.to_msg() for i in instances]
        self._pub['status'].publish(concert_msgs.SoftwareInstances(msg))

    def pub_pool_status(self):
        '''
          Publishes the list of available software in the concert.
        '''
        profiles, invalid_profiles = self._software_pool.status()
        msg = [p.to_msg() for p in profiles.values()]
        self._pub['list'].publish(concert_msgs.SoftwareProfiles(msg))

    def _print_pool_status(self):
        '''
          Logs the software pool status in console
        '''
        profiles, invalid_profiles = self._software_pool.status()

        if profiles:
            self.loginfo("===== Available Softwares =====")
            for name, profile in profiles.items():
                self.loginfo("- %s : %s"%(name, str(profile.msg.description)))
        if invalid_profiles:
            self.loginfo("===== Invalid Softwares   =====")
            for name, reason in invalid_profiles.items():
                self.loginfo("- %s : %s"%(str(name),str(reason)))
            self.loginfo("===============================")

    def loginfo(self, msg):
        '''
          Log info with Software Farm prefix

          :param msg str: message to log 
        '''
        rospy.loginfo('Software Farm : %s'%str(msg))

    def logwarn(self, msg):
        '''
          Log warn with Softwar Farm prefix
          
          :param msg str: message to log 
        '''
        rospy.logwarn('Software Farm : %s'%str(msg))
