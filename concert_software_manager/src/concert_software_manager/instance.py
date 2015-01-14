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
import concert_msgs.msg as concert_msgs

class SoftwareInstance(object):
        
    shutdown_timeout = 5
    kill_timeout = 10
    
    def __init__(self, profile):
        self._profile = profile
        self._namespace = concert_msgs.Strings.SOFTWARE_NAMESPACE  + '/' + str(self.profile.name)
        self._users = []

    def start(self):
        success = False
        try:
            force_scree = rospy.get_param(concert_msgs.Strings.PARAM_ROCON_SCREEN, True) 
            roslaunch_file_path = rocon_python_utils.ros._find_resource_from_string(self._profile.msg.launch, extension='launch')
            temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
            launch_text =self._prepare_launch_text(roslaunch_file_path, self._namespace)
            temp.write(launch_text)
            temp.close()
            self._roslaunch = roslaunch.parent.ROSLaunchParent(rospy.get_param('/run_id'), [temp.name], is_core=False, process_listeners=[], force_screen=force_screen)
            self._roslaunch._load_config()
            self._roslaunch.start()
        finally:
            if temp:
                os.unlink(temp.name)
        self._profile.counts = 1
        return success

    def stop(self):
        while self._roslaunch.pm and not self._roslaunch.pm.done:
            if count == 2 * SoftwareInstance.shutdown_timeout: 
                self._roslaunch.shutdown()
            rospy.rostime.wallsleep(0.5)
            count = count + 1

        self._profile.counts = 0
        return True

    def _prepare_launch_text(self, roslaunch_filepath, namespace):
        launch_text = '<launch>\n   <include ns="%s" file="%s">\n' % (namespace, roslaunch_filepath)
        launch_text += '</include>\n</launch>\n'

        return launch_text

    def add_user(self, user):
        if user in self._users:
            return False, self._profile.counts
        else:
            self._users.append(user)
            self._profile.counts = self._profile.counts + 1
        return True, self._profile.counts
        
    def remove_user(self, user):
        if user in self._users:
            self._users.remove(user)
            self._profile.counts = self._profile.counts - 1
            return True, self._profile.counts
        else:
            return False, self._profile.counts

    def is_max_capacity(self):
        return self._profile.counts == self._profile.max_count

    def get_namespace(self):
        return self._namespace
