#!/usr/bin/env python

'''
Created on 08/08/2011

@author: Daniel Stonier
'''
##############################################################################
# Imports
##############################################################################

import xmlrpclib

# Ros imports
import roslib; roslib.load_manifest('concert_master')
import rospy
import zeroconf_comms
from zeroconf_comms.srv import AddService

##############################################################################
# Main
##############################################################################
'''
  This currently serves the following purposes:
  
  1) Checks that the zeroconf node has come up
  2) Retrieves some parameters and advertises the master
  
  Parameters:
    ~name : name for the concert master [Concert Master].
    ~domain : zeroconf domain to advertise on [local]. 
'''
def main():
    rospy.init_node('concert_master', log_level=rospy.DEBUG)
    request = zeroconf_comms.srv.AddServiceRequest()
    request.service.name = rospy.get_param("name","Concert Master")
    request.service.type = "_concert-master._tcp"
    request.service.domain = rospy.get_param("domain","local")
    request.service.port = roslib.network.parse_http_host_and_port(roslib.rosenv.get_master_uri())[1]
    
    # Make sure the zeroconf is up and running first
    try:
        rospy.wait_for_service('add_service', 30.0)
        advertise_concert_master = rospy.ServiceProxy('add_service', zeroconf_comms.srv.AddService)
        response = advertise_concert_master(request)
        if response.result:
            rospy.loginfo("Concert Master: advertising zeroconf information [%s][%s][%s]"%(request.service.name, request.service.domain, request.service.port))
        else:
            rospy.logwarn("Concert Master : failed to advertise this concert master on the zeroconf node")
    except rospy.ROSException:
        rospy.logerr("Concert Master: timed out waiting for the zeroconf node to become available.")
        return
    
    # Find useful things to add here later.
    rospy.spin()
        
if __name__ == '__main__':
    main()
