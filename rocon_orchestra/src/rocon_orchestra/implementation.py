'''
Created on 22/11/2011

@author: snorri
'''
##############################################################################
# Imports
##############################################################################

# Ros imports
import roslib; roslib.load_manifest('rocon_orchestra')
import rospy

#from concert_msgs.srv import import *
import concert_msgs
#from concert_msgs import srv

from concert_msgs.srv import Implementation
from concert_msgs.srv import ImplementationResponse

##############################################################################
# Classes
##############################################################################

class Implementation:
    '''
      If a solution implementation is being loaded, this stores the data.
    '''
    def __init__(self):
        # This will need some modification if we go to multiple solutions on file.
        self.name = rospy.get_param("~name", "Implementation 42")
        self.nodes = rospy.get_param("~nodes", [])
        self.link_graph = rospy.get_param("~link_graph", "")
        self.device_configuration_server = rospy.Service('implementation', concert_msgs.srv.Implementation, self.serve_implementation_details)
        rospy.loginfo("Orchestration: initialised the implementation server.")

    def serve_implementation_details(self,req):
        response = concert_msgs.srv.ImplementationResponse()
        response.name = self.name
        response.nodes = self.nodes
        response.link_graph = self.link_graph
        return response
