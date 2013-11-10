#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import threading
import os
import subprocess
import unique_id

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

    data = None
    proc = None
    env = None
    thread = None

    def __init__(self, service_description=None, env=os.environ, update_callback=dummy_cb):
        '''
          @param service_description :
          @type concert_msgs.msg.ConcertService
        '''
        self.data = service_description
        self.data.uuid = unique_id.toMsg(unique_id.fromRandom())

        self.env = os.environ
        self.update_callback = update_callback

    def __del__(self):
        self.proc.kill()

    def is_enabled(self):
        return self.data.enabled

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

            while not self.data.enabled and not rospy.is_shutdown():
                self.loginfo("Waiting service to be enabled")
                rospy.sleep(1)

                if self.enable_error:
                    raise Exception(self.enable_error_message)
            rospy.logwarn("Service Manager : %s" % self.data.interactions)
            if self.data.interactions != '':
                # Can raise ResourceNotFoundException, InvalidRoleAppYaml
                role_app_loader.load(self.data.interactions, service_name=self.data.name, load=True)
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

        if self.data.enabled == False:
            success = True
            message = "Already disabled"
            return success, message

        try:
            self.proc.terminate()

            count = 0
            force_kill = False
            while self.data.enabled and not rospy.is_shutdown():
                count = count + 1
                rospy.sleep(1)

                if count == 10:  # if service does not terminate for 10 secs, force kill
                    self.loginfo("Waited too long. Force killing..")
                    self.proc.kill()
                    force_kill = True
            if self.data.interactions != '':
                # Can raise ResourceNotFoundException, InvalidRoleAppYaml
                role_app_loader.load(self.data.interactions, service_name=self.data.name, load=False)
            success = True
            message = "Force Killed" if force_kill else "Terminated"
        except (Exception, rocon_utilities.exceptions.ResourceNotFoundException, concert_roles.exceptions.InvalidRoleAppYaml) as e:
            success = False
            message = "Error while disabling : " + str(e)
        return success, message

    def run(self):
        self._enabling()

        if self.data.enabled:
            self.update_callback()
            self._wait_until_terminates()
            self.loginfo("Disabled")
            self.data.enabled = False
            self.update_callback()

    def _enabling(self):
        launcher = self.data.launcher
        ## Enabling
        try:
            self.enable_error = False
            launcher = launcher.split(" ")
            self.loginfo(launcher)
            self.proc = subprocess.Popen(launcher, env=self.env)
            self.data.enabled = True
            self.loginfo("Enabled")
        except Exception as e:
            self.loginfo(str(e))
            self.enable_error = True
            self.enable_error_message = str(e)

    def _wait_until_terminates(self):
        while not rospy.is_shutdown() and self.proc.poll() is None:
            rospy.sleep(3)

    def to_msg(self):
        return self.data

    def loginfo(self, msg):
        rospy.loginfo(str(self.data.name) + " : " + str(msg))

    def logerr(self, msg):
        rospy.logerr(str(self.data.name) + " : " + str(msg))
