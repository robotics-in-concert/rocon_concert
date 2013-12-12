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
import rocon_utilities

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
            '_request_set',
            '_lock',
            'spin',
            '_scheduler',
            '_clients',
            '_requests',
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
        self._request_set = None     # rocon_scheduler_request.transitions.RequestSet (contains ResourceReply objects)
        self._clients = {}           # common.ConcertClient.name : common.ConcertClient of all concert clients
        self._lock = threading.Lock()

        self._scheduler = rocon_scheduler_requests.Scheduler(callback=self._requester_update, topic=requests_topic_name)
        self._setup_ros_api(concert_clients_topic_name)

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
        clients = msg.clients

        for client in clients:
            if client.gateway_name not in self._clients.keys():
                rospy.loginfo("Scheduler : new concert client : %s" % client.name)
                self._clients[client.gateway_name] = common.ConcertClient(client)  # default setting is unallocated

        # @Todo : determine and handle lost clients as well.

        # should check for some things here, e.g. can we verify a client is allocated or not?
        self._update()
        self._lock.release()

    ##########################################################################
    # Requester Callback
    ##########################################################################
    # Later this should remove the need for _process_requests above.

    def _requester_update(self, request_set):
        '''
          Callback used in rocon_scheduler_requests scheduler for processing incoming resource requests.
          This gets fired every time an incoming message arrives from a Requester, which is periodically
          not upon state changes.

          @param request_set : a snapshot of all requests from a single requester in their current state.
          @type rocon_scheduler_requests.transition.RequestSet
        '''
        #for r in request_set.requests.values():
        #    print("Request: %s" % r.msg)
        self._lock.acquire()
        self._request_set = request_set
        self._update()
        self._lock.release()

    def _grant(self, request_id, resources):
        '''
          Need to update the resource information with the exact resource provided

          Note : this must be protected by being called inside locked code

          @param request_id : hex string of request id
          @type uuid hex string

          @param resources : list of resources updated with allocated concert client info
          @type scheduler_msgs.Resources[]
        '''
        for request in self._request_set.values():
            if request_id == unique_id.toHexString(request.msg.id):
                request.grant(resources)

    def _update(self):
        """
          Logic for allocating resources after there has been a state change in either the list of
          available concert clients or the incoming resource requests.

          Note : this must be protected by being called inside locked code
        """
        if self._request_set is None:
            return  # Nothing to do
        unallocated_clients = [client for client in self._clients.values() if not client.allocated]
        if not unallocated_clients:
            return  # Nothing to do
        # get all requests for compatibility tree processing and sort by priority
        # this is a bit inefficient, should just sort the request set directly? modifying it directly may be not right though
        new_requests = [r.msg for r in self._request_set.values() if r.msg.status == scheduler_msgs.Request.NEW]
        new_requests[:] = sorted(new_requests, key=lambda request: request.priority)
        last_failed_priority = None  # used to check if we should block further allocations to lower priorities
        for request in new_requests:
            if last_failed_priority is not None and request.priority < last_failed_priority:
                rospy.loginfo("Scheduler : ignoring lower priority requests until higher priorities are filled")
                break
            request_id = unique_id.toHexString(request.id)
            compatibility_tree = create_compatibility_tree(request.resources, unallocated_clients)
            compatibility_tree.print_branches("Compatibility Tree")
            pruned_branches = prune_compatibility_tree(compatibility_tree, verbosity=True)
            pruned_compatibility_tree = CompatibilityTree(pruned_branches)
            pruned_compatibility_tree.print_branches("Pruned Tree", '  ')
            if pruned_compatibility_tree.is_valid():
                rospy.loginfo("Scheduler : compatibility tree is valid, attempting to allocate...")
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
                        resource.platform_info = rocon_utilities.platform_info.set_name(leaf.msg.platform_info, leaf.msg.name)
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
                    self._grant(request_id, resources)
            else:
                last_failed_priority = request.priority
                rospy.loginfo("Scheduler : not ready yet")
