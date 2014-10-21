#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
"""
.. module:: common.concert_client

This module wraps the concert_msgs data structure for a concert client in a
python class for convenient handling inside a scheduler node.
"""
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
    """
    Envelops the concert client msg data structure that is published
    to the rest of the concert with a few extra fields and methods useful
    for management by the scheduler.
    """
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
        '''
        Initialise this concert client with the data from a msg received
        from the concert conductor.

        :param concert_msgs.ConcertClient msg:
        '''
        self.msg = msg
        """The subscribed data structure describing a concert client."""
        self.allocated = False
        """Whether or not it is currently allocated."""
        self._request_id = None
        self._resource = None
        self.allocated_priority = 0  # irrelevant while self.allocated is false
        """If allocated, this indicates its priority."""

        # aliases
        self.name = self.msg.name
        """The human readable concert alias for this client."""
        self.gateway_name = self.msg.gateway_name
        """The concert client's name on the gateway network (typically has postfixed uuid)"""

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

          :returns: the message, with updated status if allocated.
          :rtype: concert_msgs.ConcertClient
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
        '''
        Ungraceful reallocation. Stops the rapp and starts the new one.
        Should be sending a request off to the service to gracefully cancel
        the request at the time of its choosing instead.

        :param str request_id:
        :param int request_priority: usually one of ``scheduler_msgs.Request.XXX_PRIORITY`` values
        :param str resource:

        :returns: the old request id (so the requester can be updated)
        :rtype: str
        '''
        # assert checks that it has already been allocated?
        old_request_id = self._request_id
        self.abandon()
        self.allocate(request_id, request_priority, resource)  # can raise FailedToAllocateException
        return old_request_id

    def allocate(self, request_id, request_priority, resource):
        '''
        Allocate the resource. This involves also starting the rapp.

        :param str request_id:
        :param int request_priority: usually one of ``scheduler_msgs.Request.XXX_PRIORITY`` values
        :param str resource:

        :raises: :exc:`.FailedToAllocateException` if the start_rapp service failed.
        '''
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
        '''
        Abandon the resource. Usually called after a request is cancelled or on shutdown.
        This stops the rapp.
        '''
        self._stop(self.msg.gateway_name)
        self.allocated = False
        self.allocated_priority = 0  # must set this after we set allocated to false
        self._request_id = None
        self._resource = None

    def is_compatible(self, resource):
        '''
        Simple boolean test to check if the client can run the specified resource.

        :param str resource:
        :returns: true if compatible, false otherwise
        :rtype: bool
        '''
        return utils.is_compatible(self.msg, resource)

    def _start(self, gateway_name, resource):
        if self._resource == None:
            raise FailedToStartRappsException("this client hasn't been allocated yet [%s]" % self.name)
        start_rapp = rospy.ServiceProxy('/' + gateway_name.lower().replace(' ', '_') + '/start_rapp', rapp_manager_srvs.StartRapp)
        request = rapp_manager_srvs.StartRappRequest()
        request.name = resource.rapp
        request.remappings = resource.remappings
        request.parameters = resource.parameters
        try:
            start_rapp(request)
        except (rospy.service.ServiceException, rospy.exceptions.ROSInterruptException) as e:  # Service not found or ros is shutting down
            raise FailedToStartRappsException("%s" % str(e))

    def _stop(self, gateway_name):
        if self._resource == None:
            rospy.logwarn("Scheduler : this client hasn't been allocated yet, aborting stop app request  [%s]" % self.name)
            return False
        stop_rapp = rospy.ServiceProxy('/' + gateway_name.lower().replace(' ', '_') + '/stop_rapp', rapp_manager_srvs.StopRapp)
        request = rapp_manager_srvs.StopRappRequest()
        try:
            stop_rapp(request)
        except (rospy.service.ServiceException, rospy.exceptions.ROSInterruptException) as e:  # Service not found or ros is shutting down
            rospy.logwarn("Scheduler : could not stop app on '%s' [%s]" % (self.name, str(e)))
            return False
        return True
