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

from .load_params import load_parameters_from_file

##############################################################################
# Methods
##############################################################################


def dummy_cb():
    pass


class ConcertServiceInstance(object):

    __slots__ = [
            'profile',         # concert_msgs.ConcertService fixed configuration and variable parameters
            '_update_callback',    # used to trigger an external callback (service manager publisher) when the state changes.
            '_namespace',          # namespace that the service will run in
            '_lock',               # protect service enabling/disabling
            '_proc',               # holds the custom subprocess variable if TYPE_CUSTOM
            '_roslaunch',          # holds the roslaunch parent variable if TYPE_ROSLAUNCH
            '_shutdown_publisher'  # used for disabling the service
        ]

    shutdown_timeout = 5
    kill_timeout = 10

    def __init__(self, service_profile=None, env=os.environ, update_callback=dummy_cb):
        '''
          @param service_profile :
          @type concert_msgs.msg.ConcertService
        '''
        self.profile = service_profile
        self._namespace = '/services/' + str(self.profile.name)
        self._update_callback = update_callback
        self._lock = threading.Lock()
        self._proc = None
        self._roslaunch = None
        self._shutdown_publisher = rospy.Publisher(self._namespace + "/shutdown", std_msgs.Empty, latch=False)

    def __del__(self):
        if self._proc is not None:
            self._proc.kill()

    def is_enabled(self):
        return self.profile.enabled

    def enable(self, unique_identifier, interactions_loader):
        '''
        We don't need particularly complicated error codes and messages here as we don't do
        any decision making (yet) on the result. Just true/false.

        @param unique_identifier : unique id for this instance
        @type : uuid.UUID

        @param interactions_loader : used to load interactions
        @type rocon_interactions.InteractionsLoader
        '''
        self._lock.acquire()
        if self.profile.enabled:
            self._lock.release()
            return False, "already enabled"
        try:
            # Refresh the unique id
            self.profile.uuid = unique_id.toMsg(unique_identifier)
            self._start()
            if self.profile.interactions != '':
                # Can raise YamlResourceNotFoundException, MalformedInteractionsYaml
                interactions_loader.load(self.profile.interactions, namespace=self._namespace, load=True)
            # if there's a failure point, it will have thrown an exception before here.

            if self.profile.parameters != '':
                namespace = concert_msgs.Strings.SERVICE_NAMESPACE + '/' + self.profile.name
                load_parameters_from_file(self.profile.parameters, namespace, self.profile.resource, load=True)

            self.profile.enabled = True
            self._update_callback()
            self.loginfo("service enabled [%s]" % self.profile.name)
            message = "success"
        except (rocon_interactions.YamlResourceNotFoundException, rocon_interactions.MalformedInteractionsYaml) as e:
            message = "failed to enable service [%s][%s]" % (self.profile.name, str(e))
            self.logwarn(message)
        self._lock.release()
        return self.profile.enabled, message

    def disable(self, interactions_loader, unload_resources):
        '''
        @param interactions_loader : used to unload interactions.
        @type rocon_interactions.RoleAppLoader

        @param unload_resources callback to the scheduler's request resource which will unload all resources for that service.
        @type _foo_(service_name)
        '''
        success = False
        message = "unknown error"
        self._lock.acquire()
        if not self.profile.enabled:
            self._lock.release()
            return False, "already disabled"
        self._shutdown_publisher.publish(std_msgs.Empty())
        try:
            if self.profile.interactions != '':
                # Can raise YamlResourceNotFoundException, MalformedInteractionsYaml
                interactions_loader.load(self.profile.interactions, namespace=self._namespace, load=False)
            if self.profile.parameters != '':
                namespace = concert_msgs.Strings.SERVICE_NAMESPACE + '/' + self.profile.name
                load_parameters_from_file(self.profile.parameters, namespace, self.profile.name, load=False)

            launcher_type = self.profile.launcher_type
            force_kill = False

            if launcher_type == concert_msgs.ConcertService.TYPE_CUSTOM:
                count = 0
                while not rospy.is_shutdown() and self._proc.poll() is None:
                    rospy.rostime.wallsleep(0.5)
                    if count == 2 * ConcertServiceInstance.shutdown_timeout:
                        self._proc.terminate()
                    if count == 2 * ConcertServiceInstance.kill_timeout:
                        self.loginfo("waited too long, force killing..")
                        self._proc.kill()
                        force_kill = True
                        break
                    count = count + 1
            elif launcher_type == concert_msgs.ConcertService.TYPE_ROSLAUNCH:
                rospy.loginfo("Service Manager : shutting down roslaunched concert service [%s]" % self.profile.name)
                count = 0
                # give it some time to naturally die first.
                while self._roslaunch.pm and not self._roslaunch.pm.done:
                    if count == 2 * ConcertServiceInstance.shutdown_timeout:
                        self._roslaunch.shutdown()
                    rospy.rostime.wallsleep(0.5)
                    count = count + 1
            elif launcher_type == concert_msgs.ConcertService.TYPE_SHADOW:
                pass  # no processes to kill
            self.profile.enabled = False
            unload_resources(self.profile.name)
            success = True
            message = "wouldn't die so the concert got violent (force killed)" if force_kill else "died a pleasant death (terminated naturally)"
        except (rocon_interactions.YamlResourceNotFoundException, rocon_interactions.MalformedInteractionsYaml) as e:
            success = False
            message = "error while disabling [%s][%s]" % (self.profile.name, str(e))
        self._lock.release()
        return success, message

    def _start(self):

        launcher_type = self.profile.launcher_type

        if launcher_type == concert_msgs.ConcertService.TYPE_CUSTOM:
            launcher = self.profile.launcher
            launcher = launcher.split(" ")
            self._proc = subprocess.Popen(launcher)  # perhaps needs env=os.environ as an argument
        elif launcher_type == concert_msgs.ConcertService.TYPE_ROSLAUNCH:
            self._start_roslaunch()
        elif launcher_type == concert_msgs.ConcertService.TYPE_SHADOW:
            pass  # no processes to start
        else:
            # Don't care - load_service_descriptions_from_service_lists will have validated the service description file.
            pass

    def _start_roslaunch(self):
        try:
            force_screen = rospy.get_param(concert_msgs.Strings.PARAM_ROCON_SCREEN, True)
            roslaunch_file_path = rocon_python_utils.ros.find_resource_from_string(self.profile.launcher, extension='launch')
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
        return self.profile

    def loginfo(self, msg):
        rospy.loginfo("Service Manager: %s [%s]" % (str(msg), str(self.profile.name)))

    def logerr(self, msg):
        rospy.logerr("Service Manager: %s [%s]" % (str(msg), str(self.profile.name)))

    def logwarn(self, msg):
        rospy.logwarn("Service Manager: %s [%s]" % (str(msg), str(self.profile.name)))
