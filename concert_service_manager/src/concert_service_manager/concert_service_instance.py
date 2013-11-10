#!/usr/bin/env python

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

def dummy_cb():
    pass

class ConcertServiceInstance(object):

    data = None
    proc = None
    env = None
    thread = None
    namespace = None

    def __init__(self, service_description=None, env=os.environ, update_callback=dummy_cb):
        self.data = service_description
        self.data.uuid = unique_id.toMsg(unique_id.fromRandom())
        self.namespace = '/' + str(self.data.name)

        self.env = os.environ
        self.update_callback = update_callback

    def __del__(self):
        self.proc.kill()


    def is_enabled(self):
        return self.data.enabled

    def enable(self):
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

            success = True
            message = "Success"
        except Exception as e:
            success = False
            message = "Error while enabling service : " + str(e)
        return success, message 

    def disable(self):
        success = False
        message = "Not implemented"

        if self.data.enabled == False:
            success = True
            message = "Already disabled"
            return success, message

        try:
            launcher_type = self.data.launcher_type
            force_kill = False

            if launcher_type == concert_msg.ConcertService.TYPE_CUSTOM:
                self.proc.terminate()
                
                count = 0
                while self.data.enabled and not rospy.is_shutdown():
                    count = count + 1 
                    rospy.sleep(1)

                    if count == 10: # if service does not terminate for 10 secs, force kill
                        self.loginfo("Waited too long. Force killing..")
                        self.proc.kill()
                        force_kill = True
            elif launcher_type == concert_msg.ConcertService.TYPE_ROSLAUNCH:
                self.roslaunch.shutdown()

                while self.data.enabled and not rospy.is_shutdown():
                    rospy.sleep(1)
                    
            success = True
            message = "Force Killed" if force_kill else "Terminated"
        except Exception as e:
            success = False
            message = "Error while disabling : " + str(e)
            
        return success, message

    def run(self):
        self._enabling()

        self
        if self.data.enabled:
            self.update_callback()
            self._wait_until_terminates()
            self.loginfo("Disabled")
            self.data.enabled = False
            self.update_callback()

    def _enabling(self):
        ## Enabling
        try:
            self.enable_error = False
            self._start()
            self.data.enabled = True
            rospy.sleep(1)
            self.loginfo("Enabled")
        except Exception as e:
            self.loginfo(str(e))
            self.enable_error = True
            self.enable_error_message = str(e)

    def _start(self):

        launcher_type = self.data.launcher_type

        if launcher_type == concert_msg.ConcertService.TYPE_CUSTOM:
            launcher = self.data.launcher
            launcher = launcher.split(" ")
            self.proc = subprocess.Popen(launcher, env=self.env)
        elif launcher_type == concert_msg.ConcertService.TYPE_ROSLAUNCH:
            self._start_roslaunch()
        else:
            message = "Unrecognizable launcher type"
            raise Exception(message)

    def _start_roslaunch(self):
        try:
            force_screen = True
            launch_file = self.data.launcher + '.launch'
            roslaunch_file_path = rocon_utilities.find_resource_from_string(launch_file)
            temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
            launch_text = self._prepare_launch_text(roslaunch_file_path, self.namespace)
            temp.write(launch_text)
            temp.close()
            
            self.roslaunch = roslaunch.parent.ROSLaunchParent(rospy.get_param('/run_id'), [temp.name], is_core=False, process_listeners=(), force_screen=force_screen)
            self.roslaunch._load_config()
            self.roslaunch.start()
        except Exception as e:
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

        launcher_type = self.data.launcher_type

        if launcher_type == concert_msg.ConcertService.TYPE_CUSTOM:
            while not rospy.is_shutdown() and self.proc.poll() is None:
                rospy.sleep(3)
        elif launcher_type == concert_msg.ConcertService.TYPE_ROSLAUNCH:
            while self.roslaunch.pm and not self.roslaunch.pm.done:
                rospy.sleep(3)
            

    def to_msg(self):
        return self.data

    def loginfo(self, msg):
        rospy.loginfo(str(self.data.name) + " : " + str(msg))

    def logerr(self, msg):
        rospy.logerr(str(self.data.name) + " : " + str(msg))
