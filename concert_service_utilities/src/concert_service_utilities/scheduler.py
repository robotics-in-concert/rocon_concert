#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_python_comms

##############################################################################
# Methods
##############################################################################


def find_scheduler_requests_topic(timeout=rospy.rostime.Duration(5.0)):
    '''
      Do a lookup to find the scheduler requests topic.

      :param timeout: raise an exception if nothing is found before this timeout occurs.
      :type timeout: rospy.rostime.Duration

      :returns: the fully resolved name of the topic
      :rtype: str

      :raises rocon_python_comms.NotFoundException: if no topic is found within the timeout
    '''
    try:
        # assuming all topics here come in as /x/y/z/topicname or /x/y/z/topicname_355af31d
        topic_names = rocon_python_comms.find_topic('scheduler_msgs/SchedulerRequests', timeout=rospy.rostime.Duration(5.0), unique=False)
        topic_name = min(topic_names, key=len)
    except rocon_python_comms.NotFoundException:
        raise rocon_python_comms.NotFoundException("couldn't find the concert scheduler topics, aborting")
    return topic_name
