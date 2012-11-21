'''
Created on 09/08/2011

@author: snorri
'''

##############################################################################
# Imports
##############################################################################

# Ros imports
import roslib; roslib.load_manifest('rocon_conductor')
import rospy

# Rocon imports
import zeroconf_avahi
import zeroconf_msgs
from zeroconf_msgs.msg import *
from zeroconf_msgs.srv import AddListener

##############################################################################
# Functions
##############################################################################

def listen_for_app_managers():
    '''
        Tell the concert master's zeroconf node to listen for app managers.
    '''
    try:
        rospy.wait_for_service('add_listener', 30.0)
        add_listener=rospy.ServiceProxy('add_listener', AddListener)
        request = zeroconf_msgs.srv.AddListenerRequest()
        request.service_type = '_app-manager._tcp'
        response = add_listener(request)
        if response.result == False:
            rospy.logerr("Conductor: couldn't add a listener to the concert master's zeroconf node.")
            return False
    except rospy.ROSException:
        rospy.logerr("Conductor: timed out waiting for the concert master to advertise itself.")
        return False
    return True

def concert_master_name():
    '''
    Retrieve the concert master's zeroconf info. This will have automatically been started along
    with the conductor, so we just grab from the published services.
    
    Note: we assume that there is only one concert master service published on 
    this ros master!
    '''
    rospy.sleep(1.0)
    rospy.wait_for_service('list_published_services')
    try:
        try_count = 0
        while (try_count != 10):
            service_info = rospy.ServiceProxy('list_published_services', zeroconf_msgs.srv.ListPublishedServices)
            request = zeroconf_msgs.srv.ListPublishedServicesRequest()
            request.service_type = "_concert-master._tcp"
            response = service_info(request)
            if len(response.services) > 0:
                for service in response.services:
                    return service.name
            ++try_count
            rospy.sleep(1.0)
    except rospy.ServiceException, error:
        rospy.logwarn("Conductor : could not get this concert master's zeroconf details [%s]"%error)
    return None

def discover_concert_clients():
    '''
    This is a one-shot call to discover concert clients. After this we rely on the new_connections/
    lost_connections subscriber callbacks.

    Assumptions:
      1) Clients are only valid on ipv4 protocols (local lan)
    '''
    try:
        rospy.wait_for_service('list_discovered_services', timeout=5.0 )
        rospy.loginfo("Conductor: the concert master zero-configuration has been published.")
    except rospy.ROSException,e:
        rospy.logerr("Conductor: timed out waiting for the concert master's zero-configuration [%s]"%e)
        
    concert_clients = []
    try:
        discover_clients = rospy.ServiceProxy('list_discovered_services', zeroconf_msgs.srv.ListDiscoveredServices)
        request = zeroconf_msgs.srv.ListDiscoveredServicesRequest()
        request.service_type = "_app-manager._tcp"
        response = discover_clients(request)
        # Ignore ipv6 clients
        for service in response.services:
            if len(service.ipv4_addresses) > 0:
                concert_clients.append(service)
        return concert_clients
    except rospy.ServiceException, error:
        rospy.logwarn("Conductor : could not get the list of concert clients via zeroconf [%s]"%error)
