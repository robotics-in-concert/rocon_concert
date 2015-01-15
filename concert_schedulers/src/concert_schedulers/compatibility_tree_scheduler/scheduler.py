#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
"""
.. module:: compatibility_tree_scheduler.scheduler

The compatibility tree scheduler node.
"""

##############################################################################
# Imports
##############################################################################

import threading
import copy

import rospy
import unique_id
import uuid
import concert_msgs.msg as concert_msgs
import scheduler_msgs.msg as scheduler_msgs
import concert_scheduler_requests
import rocon_uri

import concert_schedulers.common as common
from concert_schedulers.common.exceptions import FailedToAllocateException
from .compatibility_tree import create_compatibility_tree, prune_compatibility_tree, CompatibilityTree
from .ros_parameters import setup_ros_parameters

##############################################################################
# Classes
##############################################################################


class CompatibilityTreeScheduler(object):
    """
    An implementation of the `concert_scheduler_requests`_ abstract Scheduler that
    uses the compatibility tree algorithm.

    It has the following characteristics:

    * attempts to resolve the highest priority request first
    * requests are resolved singly (resolving across multiple request being quite difficult)
    * if the highest priority requests are unresolved, lower priority requests are blocked
    * compatibility tree resolution of a single request works across:

     * branches - the required resource (e.g. rocon_apps/teleop)
     * leaves - concert clients that can satisfy that resource (e.g. turtlebot_1)

    * resource allocation follows the two rules:

     * always consider leaves on the lowest count branches first.
     * always take the least visible leaf away from low count branches.

    .. _`concert_scheduler_requests`: http://wiki.ros.org/concert_scheduler_requests

    """

    __slots__ = [
            '_subscribers',
            '_publishers',
            '_request_sets',
            '_lock',
            'spin',
            '_scheduler',
            '_clients',
            '_requests',
            '_parameters',
        ]

    ##########################################################################
    # Initialisation
    ##########################################################################
    def __init__(self, concert_clients_topic_name, requests_topic_name):
        '''
          :param str concert_clients_topic_name: concert client joining/leaving notifications
          :param str requests_topic_name: incoming requests for resources topic
        '''
        self._subscribers = {}       # ros subscribers
        self._publishers = {}        # ros publishers
        self._request_sets = {}      # concert_scheduler_request.transitions.RequestSet (contains ResourceReply objects)
        self._clients = {}           # common.ConcertClient.gateway_name : common.ConcertClient of all concert clients
        self._lock = threading.Lock()

        self._scheduler = concert_scheduler_requests.Scheduler(callback=self._requester_update, topic=requests_topic_name)
        self._setup_ros_api(concert_clients_topic_name)
        self._parameters = setup_ros_parameters()

        # aliases
        self.spin = rospy.spin

    def _setup_ros_api(self, concert_clients_topic_name):
        self._subscribers['concert_client_changes'] = rospy.Subscriber(concert_clients_topic_name, concert_msgs.ConcertClients, self._ros_subscriber_concert_client_changes)
        self._publishers['resource_pool'] = rospy.Publisher('~resource_pool', scheduler_msgs.KnownResources, latch=True, queue_size=10)

    ##########################################################################
    # Ros api handlers
    ##########################################################################

    def _ros_subscriber_concert_client_changes(self, msg):
        """
          Receives a list of all concert clients every time the state of the
          list changes (i.e. not periodically).

          @param msg : concert_msgs.ConcertClients
        """
        pending_notifications = []
        self._lock.acquire()
        invited_clients = msg.clients + msg.missing_clients  # both connected and missing (lost wireless connection)
        # new_clients: concert_msgs.ConcertClient[]
        new_clients = [client for client in invited_clients if client.gateway_name not in self._clients.keys()]
        # lost_clients: common.ConcertClient[]
        lost_clients = [client for client in self._clients.values() if client.gateway_name not in [c.gateway_name for c in invited_clients]]
        # work over the client list
        for client in new_clients:
            rospy.loginfo("Scheduler : new concert client [%s]" % client.name)
            self._clients[client.gateway_name] = common.ConcertClient(client)  # default setting is unallocated
        for client in lost_clients:
            if client.allocated:
                rospy.logwarn("Scheduler : lost allocated concert client [%s]" % client.name)
                client_platform_uri = rocon_uri.parse(client.msg.platform_info.uri)
                client_platform_uri.name = client.msg.gateway_name
                for request_set in self._request_sets.values():
                    found = False
                    for request in request_set.values():
                        for resource in request.msg.resources:
                            # could use a better way to check for equality than this
                            if resource.uri == str(client_platform_uri):
                                client_platform_uri.name = concert_msgs.Strings.SCHEDULER_UNALLOCATED_RESOURCE
                                resource.uri = str(client_platform_uri)
                                found = True
                                break
                        if found:
                            pending_notifications.append(request_set.requester_id)
                            # @todo might want to consider changing the request status if all resources have been unallocated
                            break
                    if found:
                        break
                    # @todo might want to validate that we unallocated since we did detect an allocated flag
            del self._clients[client.gateway_name]
        if new_clients or lost_clients:
            self._publish_resource_pool()
        # should check for some things here, e.g. can we verify a client is allocated or not?
        pending_notifications.extend(self._update(external_update=True))
        self._lock.release()
        # can't put this inside the lock scope since we can have the following deadlock scenario:
        #
        # 1) scheduler node's scheduler_requests subscriber callback thread
        # - scheduler is handling a scheduler_requests subscriber callback
        #   - BIG LOCK.acquire()
        #   - rqr.update()
        #   - user callback
        #     - CompatibilityTreeScheduler._update()
        #     - self._lock.acquire()
        # 2) this conductor client subscriber callback thread
        #   - self._lock.acquire()
        #   - scheduler.notify()
        #   - BIG LOCK.acquire()
        #
        # but does this have consequences? what if the update callback works on this request set
        # and publishes it and then we publish again here?
        for requester_id in pending_notifications:
            self._scheduler.notify(requester_id)  # publishes the request set

    def _publish_resource_pool(self):
        '''
          Publishes the current resource pool. This is called whenever the state of the scheduler's
          known resources changes.
        '''
        msg = scheduler_msgs.KnownResources()
        msg.header.stamp = rospy.Time.now()
        msg.resources = [client.toMsg() for client in self._clients.values()]
        self._publishers['resource_pool'].publish(msg)

    def _requester_update(self, request_set):
        '''
          Callback used in concert_scheduler_requests scheduler for processing incoming resource requests.
          This gets fired every time an incoming message arrives from a Requester, which is periodically
          not upon state changes.

          @param request_set : a snapshot of all requests from a single requester in their current state.
          @type concert_scheduler_requests.transition.RequestSet
        '''
        #rospy.logwarn("Scheduler : requester update callback")
        self._lock.acquire()
        # this might cause a problem if this gets blocked while in the middle of processing a lost
        # allocation (where we change the request set's resource info). Would this next line overwrite
        # our changes or would we be getting the updated request set?
        self._request_sets[request_set.requester_id.hex] = request_set
        self._update()
        self._lock.release()

    def _update(self, external_update=False):
        """
          Logic for allocating resources after there has been a state change in either the list of
          available concert clients or the incoming resource requests.

          :param external_update bool: whether or not this is called inside the scheduler thread or not.
          :returns: list of requester id's that have notifications pending.

          We have to be careful with sending notifications if calling this from outside the scheduler thread.
        """
        there_be_requests = False
        for request_set in self._request_sets.values():
            if request_set.keys():
                there_be_requests = True
        if not there_be_requests:
            return []  # nothing to do

        pending_notifications = []
        ########################################
        # Sort the request sets
        ########################################
        unallocated_clients = [client for client in self._clients.values() if not client.allocated]
        new_replies = []
        pending_replies = []
        releasing_replies = []
        for request_set in self._request_sets.values():
            new_replies.extend([r for r in request_set.values() if r.msg.status == scheduler_msgs.Request.NEW])
            pending_replies.extend([r for r in request_set.values() if r.msg.status == scheduler_msgs.Request.WAITING])
            #pending_replies.extend([r for r in request_set.values() if r.msg.status == scheduler_msgs.Request.NEW or r.msg.status == scheduler_msgs.Request.WAITING])
            releasing_replies.extend([r for r in request_set.values() if r.msg.status == scheduler_msgs.Request.CANCELING])
        # get all requests for compatibility tree processing and sort by priority
        # this is a bit inefficient, should just sort the request set directly? modifying it directly may be not right though
        pending_replies[:] = sorted(pending_replies, key=lambda request: request.msg.priority)
        ########################################
        # New
        ########################################
        for reply in new_replies:
            reply.wait()
        ########################################
        # Pending
        ########################################
        resource_pool_state_changed = False
        # used to check if we should block further allocations to lower priorities
        # important to use this variable because we might have a group of requests with equal priorities
        # and we don't want to block the whole group because the first one failed.
        last_failed_priority = None
        reallocated_clients = {}  # dic of uri string keys and request uuid hex strings
        for reply in pending_replies:
            request = reply.msg
            request_id = unique_id.toHexString(request.id)
            if last_failed_priority is not None and request.priority < last_failed_priority:
                rospy.loginfo("Scheduler : ignoring lower priority requests until higher priorities are filled")
                break
            # add preemptible clients to the candidates
            if self._parameters['enable_preemptions']:
                allocatable_clients = unallocated_clients + [client for client in self._clients.values() if (client.allocated and client.allocated_priority < request.priority)]
            else:
                allocatable_clients = unallocated_clients
            if not allocatable_clients:
                # this gets spammy...
                #rospy.loginfo("Scheduler : no resources available to satisfy request [%s]" % request_id)
                last_failed_priority = request.priority
                continue
            compatibility_tree = create_compatibility_tree(request.resources, allocatable_clients)
            if self._parameters['debug_show_compatibility_tree']:
                compatibility_tree.print_branches("Compatibility Tree")
            pruned_branches = prune_compatibility_tree(compatibility_tree, verbosity=self._parameters['debug_show_compatibility_tree'])
            pruned_compatibility_tree = CompatibilityTree(pruned_branches)
            if self._parameters['debug_show_compatibility_tree']:
                pruned_compatibility_tree.print_branches("Pruned Tree", '  ')
            if pruned_compatibility_tree.is_valid():
                rospy.loginfo("Scheduler : compatibility tree is valid, attempting to allocate [%s]" % request_id)
                last_failed_priority = None
                resources = []
                failed_to_allocate = False
                for branch in pruned_compatibility_tree.branches:
                    if failed_to_allocate:  # nested break catch
                        break
                    for leaf in branch.leaves:  # there should be but one
                        # this info is actually embedding into self._clients
                        try:
                            if leaf.allocated:
                                old_request_id = leaf.reallocate(request_id, request.priority, branch.limb)
                                reallocated_clients[leaf.msg.platform_info.uri] = old_request_id
                                rospy.loginfo("Scheduler :   pre-empted and reallocated [%s]" % leaf.name)
                            else:
                                leaf.allocate(request_id, request.priority, branch.limb)
                                rospy.loginfo("Scheduler :   allocated [%s]" % leaf.name)
                        except FailedToAllocateException as e:
                            rospy.logwarn("Scheduler :   failed to (re)allocate [%s][%s]" % (leaf.name, str(e)))
                            failed_to_allocate = True
                            break
                        resource = copy.deepcopy(branch.limb)
                        uri = rocon_uri.parse(leaf.msg.platform_info.uri)  # leaf.msg is concert_msgs/ConcertClient
                        uri.name = leaf.msg.gateway_name.lower().replace(' ', '_')  # store the unique name of the concert client
                        resource.uri = str(uri)
                        resources.append(resource)
                if failed_to_allocate:
                    rospy.logwarn("Scheduler : aborting request allocation [%s]" % request_id)
                    # aborting request allocation
                    for branch in pruned_compatibility_tree.branches:
                        for leaf in branch.leaves:  # there should be but one
                            if leaf.allocated:
                                leaf.abandon()
                    last_failed_priority = request.priority
                else:
                    reply.grant(resources)
                    resource_pool_state_changed = True
                    # remove allocated clients from the unallocated list so they don't get doubly allocated on the next request in line
                    newly_allocated_client_names = []
                    for branch in pruned_compatibility_tree.branches:
                        newly_allocated_client_names.extend([leaf.name for leaf in branch.leaves if leaf.allocated])
                    unallocated_clients[:] = [client for client in unallocated_clients if client.name not in newly_allocated_client_names]
            else:
                last_failed_priority = request.priority
                rospy.loginfo("Scheduler : insufficient resources to satisfy request [%s]" % request_id)

        ########################################
        # Preempted resource handling
        ########################################
        # this is basically equivalent to what is in the concert client changes
        # subscriber callback...can we move this somewhere centralised?
        for (resource_uri_string, request_id_hexstring) in reallocated_clients.iteritems():
            request_id = uuid.UUID(request_id_hexstring)
            # awkward that I don't have the requester id saved anywhere so
            # we have to parse through each requester's set of requests to find
            # the request id we want
            found = False
            for request_set in self._request_sets.values():
                for request in request_set.values():
                    if request.uuid == request_id:
                        for resource in request.msg.resources:
                            # could use a better way to check for equality than this
                            if resource.uri == resource_uri_string:
                                updated_resource_uri = rocon_uri.parse(resource_uri_string)
                                updated_resource_uri.name = concert_msgs.Strings.SCHEDULER_UNALLOCATED_RESOURCE
                                resource.uri = str(updated_resource_uri)
                                found = True
                                break
                    if found:
                        break
                if found:
                    if external_update:
                        pending_notifications.append(request_set.requester_id)
                    else:
                        self._scheduler.notify(request_set.requester_id)
                    # @todo might want to consider changing the request status if all resources have been unallocated
                    break

        ########################################
        # Releasing
        ########################################
        for reply in releasing_replies:
            if reply.msg.reason == scheduler_msgs.Request.NONE:
                rospy.loginfo("Scheduler : releasing resources from cancelled request [%s][none]" % ([resource.rapp for resource in reply.msg.resources]))
            elif reply.msg.reason == scheduler_msgs.Request.TIMEOUT:
                rospy.loginfo("Scheduler : releasing resources from cancelled request [%s][scheduler-requester watchdog timeout]" % ([resource.rapp for resource in reply.msg.resources]))
            else:
                rospy.loginfo("Scheduler : releasing resources from cancelled request [%s][%s]" % ([resource.rapp for resource in reply.msg.resources], reply.msg.reason))
            for resource in reply.msg.resources:
                try:
                    self._clients[rocon_uri.parse(resource.uri).name.string].abandon()
                except KeyError:
                    pass  # nothing was allocated to that resource yet (i.e. unique gateway_name was not yet set)
            reply.close()
            #reply.msg.status = scheduler_msgs.Request.RELEASED
        # Publish an update?
        if resource_pool_state_changed or releasing_replies:
            self._publish_resource_pool()
        return pending_notifications
