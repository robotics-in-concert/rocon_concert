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
import scheduler_msgs.msg as scheduler_msgs
import unique_id
import uuid
import uuid_msgs.msg as uuid_msgs

from . import utils
from .exceptions import FailedToStartRappsException, FailedToAllocateException

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
            'allocated_priority',  # priority (int) of the request it is allocated to
            '_resource',     # scheduler_msgs.Resource it fulfills
        ]

    ##########################################################################
    # Init
    ##########################################################################

    def __init__(self, msg):
        self.msg = msg
        self.allocated = False
        self._request_id = None
        self._resource = None
        self.allocated_priority = 0  # irrelevant while self.allocated is false

        # aliases
        self.name = self.msg.name
        self.gateway_name = self.msg.gateway_name

    ##########################################################################
    # Convert
    ##########################################################################

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

    def toMsg(self):
        '''
          Convert this instances to a scheduler_msgs.CurrentStatus msg type.
          The scheduler typically uses this to publish the resource on it's
          scheduler_resources_pool topic.
        '''
        msg = scheduler_msgs.CurrentStatus()
        msg.uri = self.msg.platform_info.uri
        # TODO : scheduler_msgs.CurrentStatus.MISSING
        if self.allocated:
            msg.status = scheduler_msgs.CurrentStatus.ALLOCATED
        else:
            msg.status = scheduler_msgs.CurrentStatus.AVAILABLE
        msg.owner = unique_id.toMsg(uuid.UUID(self._request_id)) if self._request_id else uuid_msgs.UniqueID()  # self._request_id is a hex string
        msg.rapps = [rapp.name for rapp in self.msg.rapps]
        return msg

    ##########################################################################
    # Allocate
    ##########################################################################

    def reallocate(self, request_id, request_priority, resource):
        # assert checks that it has already been allocated?
        old_request_id = self._request_id
        self.abandon()
        self.allocate(request_id, request_priority, resource)  # can raise FailedToAllocateException
        return old_request_id

    def allocate(self, request_id, request_priority, resource):
        self.allocated_priority = request_priority
        self.allocated = True
        self._request_id = request_id
        self._resource = resource
        try:
            self._start(self.msg.gateway_name, resource)
        except FailedToStartRappsException as e:
            self.allocated = False
            self._request_id = None
            self._resource = None
            raise FailedToAllocateException(str(e))

    def abandon(self):
        self._stop(self.msg.gateway_name)
        self.allocated = False
        self.allocated_priority = 0  # must set this after we set allocated to false
        self._request_id = None
        self._resource = None

    def is_compatible(self, resource):
        return utils.is_compatible(self.msg, resource)

    def _start(self, gateway_name, resource):
        if self._resource == None:
            raise FailedToStartRappsException("this client hasn't been allocated yet [%s]" % self.name)
        start_rapp = rospy.ServiceProxy('/' + gateway_name + '/start_rapp', rapp_manager_srvs.StartRapp)
        request = rapp_manager_srvs.StartRappRequest()
        request.name = resource.rapp
        request.remappings = resource.remappings
        try:
            start_rapp(request)
        except (rospy.service.ServiceException, rospy.exceptions.ROSInterruptException) as e:  # Service not found or ros is shutting down
            raise FailedToStartRappsException("%s" % str(e))

    def _stop(self, gateway_name):
        if self._resource == None:
            rospy.logwarn("Scheduler : this client hasn't been allocated yet, aborting stop app request  [%s]" % self.name)
            return False
        stop_rapp = rospy.ServiceProxy('/' + gateway_name + '/stop_rapp', rapp_manager_srvs.StopRapp)
        request = rapp_manager_srvs.StopRappRequest()
        try:
            stop_rapp(request)
        except (rospy.service.ServiceException, rospy.exceptions.ROSInterruptException) as e:  # Service not found or ros is shutting down
            rospy.logwarn("Scheduler : could not stop app on '%s' [%s]" % (self.name, str(e)))
            return False
        return True
