#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rosgraph
import concert_msgs.msg as concert_msgs
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import std_srvs.srv as std_srvs
import rocon_python_comms
import rocon_gateway_utils

from .concert_client import ConcertClient, ConcertClientException
from . import concert_clients
from . import exceptions
from .ros_parameters import setup_ros_parameters
from .local_gateway import LocalGateway

##############################################################################
# Conductor
##############################################################################


class Conductor(object):

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
                self.publish_conductor_graph
            )

#         self._publish_discovered_concert_clients()  # Publish an empty list, to latch it and start

    def _setup_publishers(self):
        publishers = {}
        # high frequency list_concert_clients publisher - good for connectivity statistics and app status'
        publishers["concert_clients"] = rospy.Publisher("~concert_clients", concert_msgs.ConcertClients)
        # efficient latched publisher which only publishes on client leaving/joining (and ready for action)
        publishers["concert_client_changes"] = rospy.Publisher("~concert_client_changes", concert_msgs.ConcertClients, latch=True)
        publishers["graph"] = rospy.Publisher("~graph", concert_msgs.ConductorGraph, latch=True)

        return publishers

    def publish_conductor_graph(self, clients):
        """
        :param clients dict: massive dict of dicts of all clients by state (see ConcertClient._clients variable)
        """
        # I'd like to do this: if self.publishers['graph'].get_num_connections() > 0:
        # but that means anyone introspecting it won't get to see the current state if they connected after the last change
        # and since state might not change very often, that is important.
        msg = concert_msgs.ConductorGraph()
        for state, concert_clients in clients.items():
            setattr(msg, state, [c.msg for c in concert_clients.values()])
        self.publishers['graph'].publish(msg)

    def batch_invite(self, concert_name, client_names):
        '''
          Batch invite a list of concert clients. Note we are not ever cancelling the invitations
          with this batch command.

          @param concert_name : pass our concert name in to the invitation
          @type string

          @param client_names : list of names of concert clients to invite
          @type string[]
        '''
        for name in client_names:
            try:
                # quiet abort checks
                if self._param['local_clients_only'] and not self._concert_clients[name].is_local_client:
                    continue
                # on with the invitation - rapp_manager_srvs.InviteResponse
                if self._concert_clients[name].invite(concert_name, name, cancel=False):
                    self._invited_clients[name] = True
            except KeyError:  # raised when name is not in the self._concert_clients keys
                rospy.logerr("Conductor : tried to invite unknown concert client [%s]" % name)

    def spin(self):
        '''
          Maintains the clientele list. We have to manage for two kinds here. Currently I use the same
          class interface for both with just a flag to differentiate, but it could probably use a split somewhere in the future.
        '''
        while not rospy.is_shutdown():
            remote_gateways = self._local_gateway.get_remote_gateway_info()
            self._concert_clients.update(remote_gateways)
#             # Periodic publisher
#             self._publish_discovered_concert_clients(self.publishers["concert_clients"])
#             # Long term solution - publish the changes
#             if number_of_pruned_clients != 0 or newly_ready_clients:
#                 self._publish_discovered_concert_clients()
            rospy.rostime.wallsleep(self._watcher_period)  # human time

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
            rospy.loginfo("Conductor : sending shutdown request [gateway]")
            self._local_gateway.shutdown()
            rospy.loginfo("Conductor : sending shutdown request [hub]")
            unused_response = rospy.ServiceProxy(concert_msgs.Strings.HUB_SHUTDOWN, std_srvs.Empty)()
        except rospy.ServiceException as e:
            rospy.logerr("failed to externally shut down gateway/hub [%s]" % e)

    ###########################################################################
    # Helpers
    ###########################################################################

    def _publish_discovered_concert_clients(self, list_concert_clients_publisher=None):
        '''
            Provide a list of currently discovered clients. This gets called to provide
            input to both a latched publisher for state change updates as well as a periodic
            publisher to provide continuous updates with connection statistics.

            Currently we just publish invited clients - could also publish busy or blocking
            clients, but we don't have a use case yet for that information.

            @param list_concert_clients_publisher : the publisher to use (otherwise the default latched publisher)
            @type rospy.Publisher
        '''
        discovered_concert = concert_msgs.ConcertClients()
        for unused_client_name, client in self._concert_clients.iteritems():
            try:
                if client.is_ready_for_action():
                    discovered_concert.clients.append(client.to_msg_format())
                else:
                    discovered_concert.uninvited_clients.append(client.to_msg_format())
            except ConcertClientException:
                # service was broken, quietly do not add it
                # (it will be deleted from client list next pass)
                pass
        if not list_concert_clients_publisher:
            list_concert_clients_publisher = self.publishers["concert_client_changes"]  # default
        list_concert_clients_publisher.publish(discovered_concert)
