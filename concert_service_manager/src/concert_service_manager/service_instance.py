#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import os
import subprocess
import tempfile
import threading

import roslaunch
import rocon_python_utils
import concert_msgs.msg as concert_msgs
import std_msgs.msg as std_msgs
import rocon_interactions
import unique_id

from .load_params import load_parameters_from_file, load_parameters_from_resource, load_parameters_from_key_value_msg
from .utils import *

##############################################################################
# Methods
##############################################################################


def dummy_cb():
    pass


class ServiceInstance(object):

    __slots__ = [
        'msg',                  # concert_msgs.ServiceProfile fixed and variable parameters

        '_namespace',           # namespace that the service will run in
        '_lock',                # protect service enabling/disabling
        '_proc',                # holds the custom subprocess variable if TYPE_CUSTOM
        '_roslaunch',           # holds the roslaunch parent variable if TYPE_ROSLAUNCH
        '_shutdown_publisher',  # used for disabling the service
        '_concert_name',  # todo
        '_disable_cache',  # todo
        # aliases
        'name',
    ]

    shutdown_timeout = 5
    kill_timeout = 10

    def __init__(self, concert_name=None, disable_cache=False, service_profile=None, env=os.environ):
        '''
          @param service_profile :
          @type concert_msgs.msg.ConcertService
        '''
        self._concert_name = rocon_python_utils.ros.get_ros_friendly_name(concert_name)
        self._disable_cache = disable_cache
        self.msg = service_profile
        # aliases
        self.name = self.msg.name
        # other
        self._namespace = '/services/' + str(self.msg.name)

        self._lock = threading.Lock()
        self._proc = None
        self._roslaunch = None
        self._shutdown_publisher = rospy.Publisher(self._namespace + "/shutdown", std_msgs.Empty, latch=False, queue_size=5)

    def __del__(self):
        if self._proc is not None:
            self._proc.kill()

    def enable(self, unique_identifier, interactions_loader):
        '''
        We don't need particularly complicated error codes and messages here as we don't do
        any decision making (yet) on the result. Just true/false.

        @param unique_identifier : unique id for this instance
        @type : uuid.UUID

        @param interactions_loader : used to load interactions
        @type rocon_interactions.InteractionsLoader
        '''
        success = False
        self._lock.acquire()
        try:
            # load up parameters first so that when start runs, it can find the params immediately
            if self.msg.parameters != '':
                namespace = concert_msgs.Strings.SERVICE_NAMESPACE + '/' + self.msg.name
                load_parameters_from_key_value_msg(self.msg.parameters_detail, namespace, self.msg.name, load=True)

            # Refresh the unique id
            self.msg.uuid = unique_id.toMsg(unique_identifier)
            self._start()
            if self.msg.interactions != '':
                # Can raise YamlResourceNotFoundException, MalformedInteractionsYaml
                if self._disable_cache:
                    interactions_loader.load_from_resource(self.msg.interactions, namespace=self._namespace, load=True)
                else:
                    interaction_path = os.path.join(get_service_profile_cache_home(self._concert_name, self.name), self.name + '.interactions')
                    interactions_loader.load_from_file(interaction_path, namespace=self._namespace, load=True)
            # if there's a failure point, it will have thrown an exception before here.
            success = True

            self.loginfo("service enabled [%s]" % self.msg.name)
            message = "success"
        except (rocon_interactions.YamlResourceNotFoundException, rocon_interactions.MalformedInteractionsYaml) as e:
            message = "failed to enable service [%s][%s]" % (self.msg.name, str(e))
            self.logwarn(message)
        self._lock.release()
        return success, message

    def disable(self, interactions_loader):
        '''
        @param interactions_loader : used to unload interactions.
        @type rocon_interactions.RoleAppLoader
        '''
        success = False
        message = "unknown error"
        self._lock.acquire()
        self._shutdown_publisher.publish(std_msgs.Empty())
        try:
            if self.msg.parameters != '':
                namespace = concert_msgs.Strings.SERVICE_NAMESPACE + '/' + self.msg.name
                load_parameters_from_key_value_msg(self.msg.parameters_detail, namespace, self.msg.name, load=False)

            if self.msg.interactions != '':
                # Can raise YamlResourceNotFoundException, MalformedInteractionsYaml
                if self._disable_cache:
                    interactions_loader.load_from_resource(self.msg.interactions, namespace=self._namespace, load=False)
                else:
                    interaction_path = os.path.join(get_service_profile_cache_home(self._concert_name, self.name), self.name + '.interactions')
                    interactions_loader.load_from_file(interaction_path, namespace=self._namespace, load=False)

            launcher_type = self.msg.launcher_type
            force_kill = False

            if launcher_type == concert_msgs.ServiceProfile.TYPE_CUSTOM:
                count = 0
                while not rospy.is_shutdown() and self._proc.poll() is None:
                    rospy.rostime.wallsleep(0.5)
                    if count == 2 * ServiceInstance.shutdown_timeout:
                        self._proc.terminate()
                    if count == 2 * ServiceInstance.kill_timeout:
                        self.loginfo("waited too long, force killing..")
                        self._proc.kill()
                        force_kill = True
                        break
                    count = count + 1
            elif launcher_type == concert_msgs.ServiceProfile.TYPE_ROSLAUNCH:
                rospy.loginfo("Service Manager : shutting down roslaunched concert service [%s]" % self.msg.name)
                count = 0
                # give it some time to naturally die first.
                while self._roslaunch.pm and not self._roslaunch.pm.done:
                    if count == 2 * ServiceInstance.shutdown_timeout:
                        self._roslaunch.shutdown()
                    rospy.rostime.wallsleep(0.5)
                    count = count + 1
            elif launcher_type == concert_msgs.ServiceProfile.TYPE_SHADOW:
                pass  # no processes to kill
            success = True
            message = "wouldn't die so the concert got violent (force killed)" if force_kill else "died a pleasant death (terminated naturally)"
        except (rocon_interactions.YamlResourceNotFoundException, rocon_interactions.MalformedInteractionsYaml) as e:
            success = False
            message = "error while disabling [%s][%s]" % (self.msg.name, str(e))
        self._lock.release()
        return success, message

    def _start(self):

        launcher_type = self.msg.launcher_type

        if launcher_type == concert_msgs.ServiceProfile.TYPE_CUSTOM:
            launcher = self.msg.launcher
            launcher = launcher.split(" ")
            self._proc = subprocess.Popen(launcher)  # perhaps needs env=os.environ as an argument
        elif launcher_type == concert_msgs.ServiceProfile.TYPE_ROSLAUNCH:
            self._start_roslaunch()
        elif launcher_type == concert_msgs.ServiceProfile.TYPE_SHADOW:
            pass  # no processes to start
        else:
            # Don't care - load_service_descriptions_from_service_lists will have validated the service description file.
            pass

    def _start_roslaunch(self):
        try:
            force_screen = rospy.get_param(concert_msgs.Strings.PARAM_ROCON_SCREEN, True)
            roslaunch_file_path = rocon_python_utils.ros.find_resource_from_string(self.msg.launcher, extension='launch')
            temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
            launch_text = self._prepare_launch_text(roslaunch_file_path, self._namespace)
            temp.write(launch_text)
            temp.close()
            self._roslaunch = roslaunch.parent.ROSLaunchParent(rospy.get_param('/run_id'), [temp.name], is_core=False, process_listeners=[], force_screen=force_screen)
            self._roslaunch._load_config()
            self._roslaunch.start()
        # handle properly case by case instead of using the catchall technique
#        except Exception as e:
#            import sys
#            import traceback
#            traceback.print_exc(file=sys.stdout)
#            message = "Error while roslaunching.. " + str(e)
#            raise Exception(message)
        finally:
            if temp:
                os.unlink(temp.name)

    def _prepare_launch_text(self, roslaunch_filepath, namespace):
        launch_text = '<launch>\n   <include ns="%s" file="%s">\n' % (namespace, roslaunch_filepath)
        launch_text += '</include>\n</launch>\n'

        return launch_text

    def to_msg(self):
        return self.msg

    def loginfo(self, msg):
        rospy.loginfo("Service Manager : %s [%s]" % (str(msg), str(self.msg.name)))

    def logerr(self, msg):
        rospy.logerr("Service Manager : %s [%s]" % (str(msg), str(self.msg.name)))

    def logwarn(self, msg):
        rospy.logwarn("Service Manager : %s [%s]" % (str(msg), str(self.msg.name)))
