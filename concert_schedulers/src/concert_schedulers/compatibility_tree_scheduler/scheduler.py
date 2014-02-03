#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import threading
import copy
import unique_id
import concert_msgs.msg as concert_msgs
import scheduler_msgs.msg as scheduler_msgs
import rocon_scheduler_requests
import rocon_uri

# local imports
import concert_schedulers.common as common
from concert_schedulers.common.exceptions import FailedToAllocateException
from .compatibility_tree import create_compatibility_tree, prune_compatibility_tree, CompatibilityTree

##############################################################################
# Classes
##############################################################################


class CompatibilityTreeScheduler(object):

    __slots__ = [
            '_subscribers',
            '_publishers',
            '_request_sets',
            '_lock',
            'spin',
            '_scheduler',
            '_clients',
            '_requests',
            '_debug_show_compatibility_tree'
        ]

    ##########################################################################
    # Initialisation
    ##########################################################################
    def __init__(self, concert_clients_topic_name, requests_topic_name):
        '''
          @param concert_clients_topic_name : concert client joining/leaving notifications
          @type string

          @param requests_topic_name : incoming requests for resources topic
          @type string
        '''
        self._subscribers = {}       # ros subscribers
        self._request_sets = {}     # rocon_scheduler_request.transitions.RequestSet (contains ResourceReply objects)
        self._clients = {}           # common.ConcertClient.gateway_name : common.ConcertClient of all concert clients
        self._lock = threading.Lock()

        self._scheduler = rocon_scheduler_requests.Scheduler(callback=self._requester_update, topic=requests_topic_name)
        self._setup_ros_api(concert_clients_topic_name)
        self._debug_show_compatibility_tree = True

        # aliases
        self.spin = rospy.spin

    def _setup_ros_api(self, concert_clients_topic_name):
        self._subscribers['concert_client_changes'] = rospy.Subscriber(concert_clients_topic_name, concert_msgs.ConcertClients, self._ros_subscriber_concert_client_changes)

    ##########################################################################
    # Ros callbacks
    ##########################################################################

    def _ros_subscriber_concert_client_changes(self, msg):
        """
          Receives a list of all concert clients every time the state of the
          list changes (i.e. not periodically)

          @param msg : concert_msgs.ConcertClients
        """
        self._lock.acquire()
        # new_clients: concert_msgs.ConcertClient[]
        new_clients = [client for client in msg.clients if client.gateway_name not in self._clients.keys()]
        # lost_clients: common.ConcertClient[]
        lost_clients = [client for client in self._clients.values() if client.gateway_name not in [c.gateway_name for c in msg.clients]]
        # work over the client list
        for client in new_clients:
            rospy.loginfo("Scheduler : new concert client [%s]" % client.name)
            self._clients[client.gateway_name] = common.ConcertClient(client)  # default setting is unallocated
        for client in lost_clients:
            if client.allocated:
                rospy.logwarn("Scheduler : lost allocated concert client [%s]" % client.name)
                # WARNINGg, this code actually doesn't work - it doesn't force the scheduler to send the updated information
                # back to the requester. Instead, the requester sends its heartbeat updates, which end up overwriting these
                # changes in _requester_update.
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
                            self._scheduler.notify(request_set.requester_id)
                            # @todo might want to consider changing the request status if all resources have been unallocated
                            break
                    if found:
                        break
                    # @todo might want to validate that we unallocated since we did detect an allocated flag
            del self._clients[client.gateway_name]
        # should check for some things here, e.g. can we verify a client is allocated or not?
        self._update()
        self._lock.release()

    def _requester_update(self, request_set):
        '''
          Callback used in rocon_scheduler_requests scheduler for processing incoming resource requests.
          This gets fired every time an incoming message arrives from a Requester, which is periodically
          not upon state changes.

          @param request_set : a snapshot of all requests from a single requester in their current state.
          @type rocon_scheduler_requests.transition.RequestSet
        '''
        #rospy.logwarn("Scheduler : requester update callback")
        self._lock.acquire()
        # this might cause a problem if this gets blocked while in the middle of processing a lost
        # allocation (where we change the request set's resource info). Would this next line overwrite
        # our changes or would we be getting the updated request set?
        self._request_sets[request_set.requester_id.hex] = request_set
        self._update()
        self._lock.release()

    def _update(self):
        """
          Logic for allocating resources after there has been a state change in either the list of
          available concert clients or the incoming resource requests.

          Note : this must be protected by being called inside locked code

          @todo test use of rlocks here
        """
        there_be_requests = False
        for request_set in self._request_sets.values():
            if request_set.keys():
                there_be_requests = True
        if not there_be_requests:
            return  # Nothing to do

        ########################################
        # Sort the request sets
        ########################################
        unallocated_clients = [client for client in self._clients.values() if not client.allocated]
        new_replies = []
        pending_replies = []
        releasing_replies = []
        for request_set in self._request_sets.values():
            new_replies.extend([r for r in request_set.values() if r.msg.status == scheduler_msgs.Request.NEW])
            pending_replies.extend([r for r in request_set.values() if r.msg.status == scheduler_msgs.Request.NEW or r.msg.status == scheduler_msgs.Request.WAITING])
            releasing_replies.extend([r for r in request_set.values() if r.msg.status == scheduler_msgs.Request.CANCELING])
        # get all requests for compatibility tree processing and sort by priority
        # this is a bit inefficient, should just sort the request set directly? modifying it directly may be not right though
        pending_replies[:] = sorted(pending_replies, key=lambda request: request.msg.priority)
        ########################################
        # No allocations
        ########################################
        if not unallocated_clients and new_replies:
            for reply in new_replies:
                # should do something here (maybe validation)?
                reply.wait()
        ########################################
        # Allocations
        ########################################
        if unallocated_clients:
            last_failed_priority = None  # used to check if we should block further allocations to lower priorities
            for reply in pending_replies:
                request = reply.msg
                if last_failed_priority is not None and request.priority < last_failed_priority:
                    rospy.loginfo("Scheduler : ignoring lower priority requests until higher priorities are filled")
                    break
                request_id = unique_id.toHexString(request.id)
                compatibility_tree = create_compatibility_tree(request.resources, unallocated_clients)
                if self._debug_show_compatibility_tree:
                    compatibility_tree.print_branches("Compatibility Tree")
                pruned_branches = prune_compatibility_tree(compatibility_tree, verbosity=self._debug_show_compatibility_tree)
                pruned_compatibility_tree = CompatibilityTree(pruned_branches)
                if self._debug_show_compatibility_tree:
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
                                leaf.allocate(request_id, branch.limb)
                                rospy.loginfo("Scheduler :   allocated [%s]" % leaf.name)
                            except FailedToAllocateException as e:
                                rospy.logwarn("Scheduler :   failed to allocate [%s][%s]" % (leaf.name, str(e)))
                                failed_to_allocate = True
                                break
                            resource = copy.deepcopy(branch.limb)
                            uri = rocon_uri.parse(leaf.msg.platform_info.uri)  # leaf.msg is concert_msgs/ConcertClient
                            uri.name = leaf.msg.gateway_name  # store the unique name of the concert client
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
                        if reply.msg.status == scheduler_msgs.Request.NEW:
                            reply.wait()
                    else:
                        reply.grant(resources)
                else:
                    if reply.msg.status == scheduler_msgs.Request.NEW:
                        reply.wait()
                    last_failed_priority = request.priority
                    rospy.loginfo("Scheduler : insufficient resources to satisfy request [%s]" % request_id)
        ########################################
        # Releasing Allocated Concert Clients
        ########################################
        for reply in releasing_replies:
            #print("Releasing Resources: %s" % [r.rapp for r in reply.msg.resources])
            #print("  Releasing Resources: %s" % [r.uri for r in reply.msg.resources])
            #print("  Clients: %s" % self._clients.keys())
            for client in self._clients.values():
                print(str(client))
            for resource in reply.msg.resources:
                try:
                    self._clients[rocon_uri.parse(resource.uri).name.string].abandon()
                except KeyError:
                    pass  # nothing was allocated to that resource yet (i.e. unique gateway_name was not yet set)
            reply.close()
            #reply.msg.status = scheduler_msgs.Request.RELEASED
