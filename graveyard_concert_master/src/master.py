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
import rosgraph
import rospy
import zeroconf_msgs
from zeroconf_msgs.srv import AddService

##############################################################################
# Functions
##############################################################################

try:
    import urllib.parse as urlparse
except ImportError:
    import urlparse

def parse_http_host_and_port(url):     
    """     
    Convenience routine to handle parsing and validation of HTTP URL     
    port due to the fact that Python only provides easy accessors in     
    Python 2.5 and later. Validation checks that the protocol and host     
    are set.     
     
    @param url: URL to parse     
    @type  url: str     
    @return: hostname and port number in URL or 80 (default).     
    @rtype: (str, int)     
    @raise ValueError: if the url does not validate     
    """     
    # can't use p.port because that's only available in Python 2.5     
    if not url:     
        raise ValueError('not a valid URL')     
    p = urlparse.urlparse(url)     
    if not p[0] or not p[1]: #protocol and host     
        raise ValueError('not a valid URL')     
    if ':' in p[1]:     
        hostname, port = p[1].split(':')     
        port = int(port)     
    else:     
        hostname, port = p[1], 80     
    return hostname, port

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
    request = zeroconf_msgs.srv.AddServiceRequest()
    request.service.name = rospy.get_param("name","Concert Master")
    request.service.type = "_concert-master._tcp"
    request.service.domain = rospy.get_param("domain","local")
    request.service.port = rosgraph.network.parse_http_host_and_port(rosgraph.rosenv.get_master_uri())[1]

    # Make sure the zeroconf is up and running first
    try:
        rospy.wait_for_service('add_service', 30.0)
        advertise_concert_master = rospy.ServiceProxy('add_service', zeroconf_msgs.srv.AddService)
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
