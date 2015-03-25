#!/usr/bin/env python

import rospy
import rocon_python_comms
import concert_msgs.srv as concert_srvs
import concert_msgs.msg as concert_msgs

"""
class FailedToStartSoftwareException(Exception):
    '''
        Exception to handle failure of Software startup
    '''
    pass
"""

class SoftwareFarmClient(object):
    '''
        A client-side python module to interact with Concert Software Farmer(server-side).
    '''

    def __init__(self):
        '''
        Search for handle of concert software farm and get ready to serve high level system's requests
        '''
        software_farm_srv_name = rocon_python_comms.find_service('concert_msgs/AllocateSoftware', timeout=rospy.rostime.Duration(5.0), unique=True)
        self._software_farm_srv = rospy.ServiceProxy(software_farm_srv_name, concert_srvs.AllocateSoftware)

    def allocate(self, software_name):
        '''
          Sends allocation requets to Concert Software Farmer. 

          :param str software_name: The name of softare to allocate. It is resource tuple. e.g) 'concert_software_common/world_canvas_server'
         
          :returns: whether it is successfull, the software namespace, and its current parameter configuration
          :rtype: bool, str, rocon_std_msgs/KeyValue[]
        '''
        return self._request_farmer(software_name, True)

    def deallocate(self, software_name):
        '''
          Sends deallocation requets to Concert Software Farmer. 

          :param str software_name: The name of softare to deallocate. It is resource tuple. e.g) 'concert_software_common/world_canvas_server'
         
          :returns: whether it is successfull, the software namespace, and its current parameter configuration
          :rtype: bool, str, rocon_std_msgs/KeyValue[]
        '''
        return self._request_farmer(software_name, False)

    def _request_farmer(self, software_name, enable):
        '''
          Generates request message and invoke rosservice call to acutally allocate software.
        
          :param str software_name: The software name 
          :param bool enable: whether software need to be allocated or deallocated

          :returns: whether it is successfull, the software namespace, and its current parameter configuration
          :rtype: bool, str, rocon_std_msgs/KeyValue[]
        '''
        req = concert_srvs.AllocateSoftwareRequest()
        req.user = rospy.get_name()
        req.software = software_name 
        req.allocate = enable 
        resp = self._software_farm_srv(req)
 
        return resp.success, resp.namespace
