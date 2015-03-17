#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
###############################################################################

import rospy

import std_msgs.msg as std_msgs
from qt_dotgraph.pydotfactory import PydotFactory

from .dotcode import ConductorGraphDotcodeGenerator
from .conductor_graph_info import ConductorGraphInfo


class ConductorGraphDotcodeToString():

    def __init__(self, clusters=False):

        self._clusters = clusters
        self._dotcode_factory = PydotFactory()
        self._dotcode_generator = ConductorGraphDotcodeGenerator()
        self._pub_string = None

        self._graph = ConductorGraphInfo(change_callback=self._update_conductor_graph, periodic_callback=self._periodic_callback)

        while not self._graph.is_conductor and not rospy.is_shutdown():
            #self.loginfo("waits for conductor to be ready.")
            rospy.rostime.wallsleep(1)

        self._pub_string = rospy.Publisher(self._graph.namespace + '/graph_string', std_msgs.String, queue_size=2)

    def _update_conductor_graph(self):
        current_dotcode = self._dotcode_generator.generate_dotcode(conductor_graph_instance=self._graph, dotcode_factory=self._dotcode_factory, clusters=self._clusters)
        if self._pub_string:
            self._pub_string.publish(str(current_dotcode))

    def _periodic_callback(self):
        pass

    def spin(self):
        rospy.spin()

    def loginfo(self, msg):
        rospy.loginfo("Conductor Graph To String : %s"%msg)
