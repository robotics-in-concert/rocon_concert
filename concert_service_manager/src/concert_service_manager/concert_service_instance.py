#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import traceback
import threading
import os
import subprocess
import unique_id
import roslaunch
import rocon_utilities
import tempfile

import uuid_msgs.msg as uuid_msg
import concert_msgs.msg as concert_msg
import concert_msgs.srv as concert_srv
import rocon_utilities
import concert_roles

##############################################################################
# Methods
##############################################################################


def dummy_cb():
    pass


class ConcertServiceInstance(object):

    proc = None
    env = None
    thread = None
    namespace = None

    def __init__(self, service_description=None, env=os.environ, update_callback=dummy_cb):
        '''
          @param service_description :
          @type concert_msgs.msg.ConcertService
        '''
        self._description = service_description
        self.namespace = '/' + str(self._description.name)

        self.env = os.environ
        self.update_callback = update_callback

    def __del__(self):
        self.proc.kill()

    def is_enabled(self):
        return self._description.enabled

    def enable(self, role_app_loader):
        '''
        @param role_app_loader : used to load role-app configurations on the role manager
        @type concert_roles.RoleAppLoader
        '''
        success = False
        message = "Not implemented"

        try:
            self.thread = threading.Thread(target=self.run)
            self.thread.start()

            while not self._description.enabled and not rospy.is_shutdown():
                self.loginfo("waiting for service to be enabled")
                rospy.sleep(1)
                if self.enable_error:
                    raise Exception(self.enable_error_message)
            self.logwarn("%s" % self._description.interactions)
            if self._description.interactions != '':
                # Can raise ResourceNotFoundException, InvalidRoleAppYaml
                role_app_loader.load(self._description.interactions, service_name=self._description.name, load=True)
            success = True
            message = "Success"
        except (Exception, rocon_utilities.exceptions.ResourceNotFoundException, concert_roles.exceptions.InvalidRoleAppYaml) as e:
            success = False
            message = "Error while enabling service : " + str(e)
        return success, message

    def disable(self, role_app_loader):
        '''
        @param role_app_loader : used to load role-app configurations on the role manager
        @type concert_roles.RoleAppLoader
        '''
        success = False
        message = "Not implemented"

        if self._description.enabled is False:
            success = True
            message = "Already disabled"
            return success, message

        try:
            if self._description.interactions != '':
                # Can raise ResourceNotFoundException, InvalidRoleAppYaml
                role_app_loader.load(self._description.interactions, service_name=self._description.name, load=False)

            launcher_type = self._description.launcher_type
            force_kill = False

            if launcher_type == concert_msg.ConcertService.TYPE_CUSTOM:
                self.proc.terminate()

                count = 0
                while self._description.enabled and not rospy.is_shutdown():
                    count = count + 1
                    rospy.sleep(1)

                    if count == 10:  # if service does not terminate for 10 secs, force kill
                        self.loginfo("Waited too long. Force killing..")
                        self.proc.kill()
                        force_kill = True
            elif launcher_type == concert_msg.ConcertService.TYPE_ROSLAUNCH:
                self.roslaunch.shutdown()

                while self._description.enabled and not rospy.is_shutdown():
                    rospy.sleep(1)
            success = True
            message = "Force Killed" if force_kill else "Terminated"
        except (Exception, rocon_utilities.exceptions.ResourceNotFoundException, concert_roles.exceptions.InvalidRoleAppYaml) as e:
            success = False
            message = "Error while disabling : " + str(e)
        return success, message

    def run(self):
        self._enabling()

        if self._description.enabled:
            self.update_callback()
            self._wait_until_terminates()
            self.loginfo("disabled")
            self._description.enabled = False
            self.update_callback()

    def _enabling(self):
        ## Enabling
        try:
            self.enable_error = False
            self._start()
            self._description.enabled = True
            rospy.sleep(1)
            self.loginfo("Enabled")
        except Exception as e:
            self.loginfo("failed to enable %s" % str(e))
            self.enable_error = True
            self.enable_error_message = str(e)

    def _start(self):

        launcher_type = self._description.launcher_type

        if launcher_type == concert_msg.ConcertService.TYPE_CUSTOM:
            launcher = self._description.launcher
            launcher = launcher.split(" ")
            self.proc = subprocess.Popen(launcher, env=self.env)
        elif launcher_type == concert_msg.ConcertService.TYPE_ROSLAUNCH:
            self._start_roslaunch()
        else:
            message = "Unrecognizable launcher type"
            raise Exception(message)

    def _start_roslaunch(self):
        try:
            rospy.logwarn("Start roslaunch")
            force_screen = True
            roslaunch_file_path = rocon_utilities.find_resource_from_string(self._description.launcher, extension='launch', rospack=rospack)
            temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
            launch_text = self._prepare_launch_text(roslaunch_file_path, self.namespace)
            temp.write(launch_text)
            temp.close()

            self.roslaunch = roslaunch.parent.ROSLaunchParent(rospy.get_param('/run_id'), [temp.name], is_core=False, process_listeners=(), force_screen=force_screen)
            self.roslaunch._load_config()
            self.roslaunch.start()
        except Exception as e:
            import sys
            traceback.print_exc(file=sys.stdout)
            message = "Error while roslaunching.. " + str(e)
            raise Exception(message)
        finally:
            if temp:
                os.unlink(temp.name)

    def _prepare_launch_text(self, roslaunch_filepath, namespace):
        launch_text = '<launch>\n   <include ns="%s" file="%s">\n' % (namespace, roslaunch_filepath)
        launch_text += '</include>\n</launch>\n'

        return launch_text

    def _wait_until_terminates(self):

        launcher_type = self._description.launcher_type

        if launcher_type == concert_msg.ConcertService.TYPE_CUSTOM:
            while not rospy.is_shutdown() and self.proc.poll() is None:
                rospy.sleep(3)
        elif launcher_type == concert_msg.ConcertService.TYPE_ROSLAUNCH:
            while self.roslaunch.pm and not self.roslaunch.pm.done:
                rospy.sleep(3)

    def to_msg(self):
        return self._description

    def loginfo(self, msg):
        rospy.loginfo("Service Manager: %s [%s]" % (str(msg), str(self._description.name)))

    def logerr(self, msg):
        rospy.logerr("Service Manager: %s [%s]" % (str(msg), str(self._description.name)))

    def logwarn(self, msg):
        rospy.logwarn("Service Manager: %s [%s]" % (str(msg), str(self._description.name)))
