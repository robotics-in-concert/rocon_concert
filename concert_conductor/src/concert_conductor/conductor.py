#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#

"""
.. module:: conductor

This module defines the primary class for the concert conductor node.
"""

##############################################################################
# Imports
##############################################################################

import rospy
import concert_msgs.msg as concert_msgs

from .concert_client import ConcertClient
from . import concert_clients
from .ros_parameters import setup_ros_parameters
from .local_gateway import LocalGateway

##############################################################################
# Conductor
##############################################################################


class Conductor(object):
    """
    This class is the primary class for the concert conductor node. It handles
    the ros level startup, communications and shutdown.
    """

    ###########################################################################
    # Construction & Destruction
    ###########################################################################

    def __init__(self):
        """
        Initialises the conductor, but doesn't try to do anything yet.

        :raises: :exc:`.ConductorFailureException` if construction went awry.
        """

        ##################################
        # Local Information
        ##################################
        self._local_gateway = LocalGateway()  # can throw rocon_python_comms.NotFoundException.
        self._concert_name = self._local_gateway.name
        self._concert_ip = self._local_gateway.ip

        ##################################
        # Ros
        ##################################
        self.publishers = self._setup_publishers()  # can raise ConductorFailureException
        rospy.on_shutdown(self._shutdown)
        self._param = setup_ros_parameters()

        ##################################
        # Variables
        ##################################
        self._watcher_period = 1.0  # Period for the watcher thread (i.e. update rate)
        self._concert_clients = \
            concert_clients.ConcertClients(
                self._local_gateway,
                self._param,
                self.publish_concert_clients,
                self.publish_conductor_graph
            )
        self.publish_concert_clients()  # Publish an empty list, to latch it and start

    def _shutdown(self):
        """
            Last thing to do as a concert is shutting down - send an uninvite
            to all concert clients which will stop any apps currently running.

            This is usually done as a rospy shutdown hook.
        """
        # Don't worry about forcing the spin loop to come to a closure - rospy basically puts a halt
        # on it at the rospy.rostime call once we enter the twilight zone (shutdown hook period).
        self._concert_clients.shutdown()
        try:
            rospy.loginfo("Conductor : sending shutdown request [gateway/hub]")
            self._local_gateway.shutdown()
        except rospy.ServiceException as e:
            rospy.logerr("failed to externally shut down gateway/hub [%s]" % e)

    ###########################################################################
    # Runtime
    ###########################################################################

    def spin(self):
        '''
          Maintains the clientele list. We have to manage for two kinds here. Currently I use the same
          class interface for both with just a flag to differentiate, but it could probably use a split somewhere in the future.
        '''
        while not rospy.is_shutdown():
            remote_gateways = self._local_gateway.get_remote_gateway_info()
            self._concert_clients.update(remote_gateways)
            # Periodic publisher
#             # Long term solution - publish the changes
#             if number_of_pruned_clients != 0 or newly_ready_clients:
#                 self._publish_discovered_concert_clients()
            rospy.rostime.wallsleep(self._watcher_period)  # human time

    ###########################################################################
    # Publishers
    ###########################################################################

    def _setup_publishers(self):
        publishers = {}
        # high frequency list_concert_clients publisher - good for connectivity statistics and app status'
        publishers["concert_clients"] = rospy.Publisher("~concert_clients", concert_msgs.ConcertClients, queue_size=5)
        # efficient latched publisher which only publishes on client leaving/joining (and ready for action)
        publishers["concert_client_changes"] = rospy.Publisher("~concert_client_changes", concert_msgs.ConcertClients, latch=True, queue_size=1)
        publishers["graph"] = rospy.Publisher("~graph", concert_msgs.ConductorGraph, latch=True, queue_size=5)
        return publishers

    def publish_conductor_graph(self, clients):
        """
        Publish the full list of concert clients (i.e. clients from every state, including bad, gone). This publication
        is typically only intended for introspecting tools.

        :param clients dict: massive dict of dicts of all clients by state (see ConcertClient._clients variable)
        """
        # I'd like to do this: if self.publishers['graph'].get_num_connections() > 0:
        # but that means anyone introspecting it won't get to see the current state if they connected after the last change
        # and since state might not change very often, that is important.
        msg = concert_msgs.ConductorGraph()
        for state, concert_clients in clients.items():
            setattr(msg, state, [c.msg for c in concert_clients.values()])
        self.publishers['graph'].publish(msg)

    def publish_concert_clients(self, clients={}, changes_only=True):
        '''
        Provide a list of currently discovered clients. This gets called to provide
        input to both a latched publisher for state change updates as well as a periodic
        publisher to provide continuous updates with connection statistics.

        :param clients dict: massive dict of dicts of all clients by state (see ConcertClient._clients variable)
        :param changes_only bool: publish on the changes only topic, or the periodically published topic
        :param publisher rospy.Publisher: the publisher to use (otherwise the default latched publisher)
        '''
        msg = concert_msgs.ConcertClients()
        if clients:  # don't add anything if it's empty (just initiating the latched publisher
            for concert_client in clients[ConcertClient.State.UNINVITED].values():
                msg.uninvited_clients.append(concert_client.msg)
            for concert_client in clients[ConcertClient.State.MISSING].values():
                msg.missing_clients.append(concert_client.msg)
            for concert_client in clients[ConcertClient.State.AVAILABLE].values():
                msg.clients.append(concert_client.msg)
        if changes_only:
            publisher = self.publishers["concert_client_changes"]  # default
        else:
            publisher = self.publishers["concert_clients"]  # default
        publisher.publish(msg)
