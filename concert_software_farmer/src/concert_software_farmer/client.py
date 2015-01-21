#!/usr/bin/env python

import rospy
import rocon_python_comms
import concert_msgs.srv as concert_srvs
import concert_msgs.msg as concert_msgs

class FailedToStartSoftwareException(Exception):
    pass

class SoftwareFarmClient(object):

    def __init__(self):
        self.software_farm_srv_name = rocon_python_comms.find_service('concert_msgs/AllocateSoftware', timeout=rospy.rostime.Duration(5.0), unique=True)
        self.software_farm_srv = rospy.ServiceProxy(software_farm_srv_name, concert_srvs.AllocateSoftware)

    def allocate(self, software_name):
        namespace = self._request_farmer(software_name, True)
        return namespace

    def deallocate(self, software_name):
        return self._request_farmer(software_name, False)

    def _request_farmer(self, software_name, enable):
        req = concert_srvs.AllocateSoftwareRequest()
        req.user = rospy.get_name()
        req.software = software_name 
        req.allocate = enable 
        resp = software_farm_handle(req)
 
        if resp.success:
            return resp.success
        else:
            raise FailedToStartSoftwareException("Failed to start world canvas")
