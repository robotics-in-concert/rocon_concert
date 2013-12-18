#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_app_manager_msgs.srv as rapp_manager_srvs

# local imports
import utils
from exceptions import FailedToStartAppsException, FailedToAllocateException

##############################################################################
# Classes
##############################################################################


class ConcertClient(object):
    __slots__ = [
            'msg',           # concert_msgs/ConcertClient
            'name',          # alias to msg.name
            'gateway_name',  # alias to msg.gateway_name
            'allocated',     # boolean value representing whether it has been allocated or not.
            '_request_id',   # id (uuid hex string) of the request it is allocated to
            '_resource',     # scheduler_msgs.Resource it fulfills
        ]

    def __init__(self, msg):
        self.msg = msg
        self.allocated = False
        self._request_id = None
        self._resource = None

        # aliases
        self.name = self.msg.name
        self.gateway_name = self.msg.gateway_name

    def __str__(self):
        rval = "Concert Client\n"
        rval += "  Name: %s\n" % self.name
        rval += "  Gateway Name: %s\n" % self.gateway_name
        if self.allocated:
            rval += "  Allocated: yes\n"
            rval += "  Request Id: %s\n" % self._request_id
            #rval += "  Resource: %s" % self._resource
        else:
            rval += "  Allocated: no\n"
        return rval

    def allocate(self, request_id, resource):
        self.allocated = True
        self._request_id = request_id
        self._resource = resource
        try:
            self._start(self.msg.gateway_name, resource)
        except FailedToStartAppsException as e:
            self.allocated = False
            self._request_id = None
            self._resource = None
            raise FailedToAllocateException(str(e))

    def abandon(self):
        self._stop(self.msg.gateway_name)
        self.allocated = False
        self._request_id = None
        self._resource = None

    def is_compatible(self, resource):
        return utils.is_compatible(self.msg, resource)

    def _start(self, gateway_name, resource):
        if self._resource == None:
            raise FailedToStartAppsException("this client hasn't been allocated yet [%s]" % self.name)
        start_app = rospy.ServiceProxy('/' + gateway_name + '/start_app', rapp_manager_srvs.StartApp)
        request = rapp_manager_srvs.StartAppRequest()
        request.name = resource.name
        request.remappings = resource.remappings
        try:
            start_app(request)
        except (rospy.service.ServiceException, rospy.exceptions.ROSInterruptException) as e:  # Service not found or ros is shutting down
            raise FailedToStartAppsException("%s" % str(e))

    def _stop(self, gateway_name):
        if self._resource == None:
            rospy.logwarn("Scheduler : this client hasn't been allocated yet, aborting stop app request  [%s]" % self.name)
            return False
        stop_app = rospy.ServiceProxy('/' + gateway_name + '/stop_app', rapp_manager_srvs.StopApp)
        request = rapp_manager_srvs.StopAppRequest()
        try:
            stop_app(request)
        except (rospy.service.ServiceException, rospy.exceptions.ROSInterruptException) as e:  # Service not found or ros is shutting down
            rospy.logwarn("Scheduler : could not stop app '%s' on '%s' [%s]" % (resource.name, self.name, str(e)))
            return False
        return True
