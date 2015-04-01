#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Module
##############################################################################
"""
.. module:: local_gateway

This module exposes interactions with the local gateway.
"""
##############################################################################
# Imports
##############################################################################

import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import rocon_python_comms
import rospy
import std_srvs.srv as std_srvs

##############################################################################
# Classes
##############################################################################


class LocalGateway(object):
    """
    Convenience class for discovering and interacting with the local
    gateway node.
    """
    __slots__ = [
        '_services',
        'name',
        'ip',
    ]

    def __init__(self):
        """
        This resolves the local gateway, blocking until it returns with the gateway information.

        :raises: :exc:`.rocon_python_comms.NotFoundException` if services couldn't be found.
        """
        try:
            self._services = self._setup_ros_services()
            (self.name, self.ip) = self._get_gateway_info()
        except rocon_python_comms.NotFoundException:
            raise

    def _setup_ros_services(self):
        """
        :raises: :exc:`.rocon_python_comms.NotFoundException` if services couldn't be found.
        """
        services = {}
        services['pull']                = rospy.ServiceProxy('~gateway_pull', gateway_srvs.Remote, persistent=True)  # @IgnorePep8 noqa
        services['remote_gateway_info'] = rospy.ServiceProxy("~remote_gateway_info", gateway_srvs.RemoteGatewayInfo, persistent=True)
        try:
            for service in services.values():
                service.wait_for_service(10.0)  # can throw rospy.ServiceException
        except rospy.ServiceException:
            raise rocon_python_comms.NotFoundException("couldn't find the local gateway services")
        return services

    def shutdown(self):
        """
        Initiates a shutdown call to both gateway and hub. This is executed as a ros shutdown
        hook by the concert conductor class. This is necessary so the hub and gateway can
        clean up and transmit final updates to all concert client gateways.

        .. seealso:: :class:`.ConcertConductor`
        """
        unused_response = rospy.ServiceProxy("~gateway_shutdown", std_srvs.Empty)()
        unused_response = rospy.ServiceProxy('~hub_shutdown', std_srvs.Empty)()

    def _get_gateway_info(self):
        gateway_info_proxy = rocon_python_comms.SubscriberProxy("~gateway_info", gateway_msgs.GatewayInfo)
        # This needs to be in a loop, since it must not only check for a response, but that the gateway
        # is connected to the hub. If it isn't connected, it needs to try again.
        start_time = rospy.get_rostime()
        while not rospy.is_shutdown():
            gateway_info = gateway_info_proxy(rospy.Duration(0.1))
            if gateway_info:
                if gateway_info.connected:
                    name = gateway_info.name
                    ip = gateway_info.ip
                    break
                else:
                    rospy.loginfo("Conductor : no hub yet available, spinning...")
                    rospy.rostime.wallsleep(0.1)
            if rospy.get_rostime() - start_time > rospy.Duration(15.0):
                raise rocon_python_comms.NotFoundException("couldn't retrieve gateway information")
        gateway_info_proxy.unregister()
        return (name, ip)

    def get_remote_gateway_info(self):
        """
        Calls the remote gateway info service and returns information on the remote gateways.

        :return: a list of the remote gateways or empty list if there was a service error.
        :rtype gateway_msgs.RemoteGateway[]
        """
        try:
            remote_gateway_info = self._services['remote_gateway_info']()
        except rospy.service.ServiceException:  # service not available
            return []
        except rospy.exceptions.ROSInterruptException:  # ros shutdown
            return []
        return remote_gateway_info.gateways

    def request_pulls(self, remote_gateway_name, cancel=False, service_names=['platform_info', 'list_rapps', 'invite'], topic_names=['status']):
        """
        Handles pull requests and cancels from request gateways for the conductor. Note this
        only applies to topics/services relevant for interacting with concert clients.

        :param str remote_gateway_name: name of a remote gateway to apply to all rules
        :param bool cancel: to register or unregister the pull requests
        """
        req = gateway_srvs.RemoteRequest()
        req.cancel = cancel
        req.remotes = []
        for service_name in service_names:
            rule = gateway_msgs.Rule()
            rule.name = str('/' + remote_gateway_name.lower().replace(' ', '_') + '/' + service_name)
            rule.node = ''
            rule.type = gateway_msgs.ConnectionType.SERVICE
            req.remotes.append(gateway_msgs.RemoteRule(remote_gateway_name.lstrip('/'), rule))
        for publisher_name in topic_names:
            rule = gateway_msgs.Rule()
            rule.name = str('/' + remote_gateway_name.lower().replace(' ', '_') + '/' + publisher_name)
            rule.node = ''
            rule.type = gateway_msgs.ConnectionType.PUBLISHER
            req.remotes.append(gateway_msgs.RemoteRule(remote_gateway_name.lstrip('/'), rule))
        # TODO : exception handling for this call
        response = self._services['pull'](req)
        if response.result != gateway_msgs.ErrorCodes.SUCCESS and not cancel:  # don't worry about errors on cleanup
            rospy.logwarn("Conductor: failed to register pull requests from the concert client [%s]%s" % (remote_gateway_name, service_names))  # TODO : exceptions, but what kind of failures?
