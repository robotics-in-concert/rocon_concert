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
import unique_id
import concert_msgs.msg as concert_msgs
import rocon_scheduler_requests

import impl

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
        self._clients = {}           # impl.ConcertClient.name : impl.ConcertClient of all concert clients
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
            @param
                msg : concert_msgs.ConcertClients

            1. Stops services which client left.
            2. Rebuild client list
            3. Starts services which has all requested resources
        """
        self._lock.acquire()
        clients = msg.clients

        for client in clients:
            rospy.loginfo("Client : %s" % client.name)
            if client.gateway_name not in self._clients.keys():
                rospy.loginfo("  New Client : %s" % client.name)
                self._clients[client.gateway_name] = impl.ConcertClient(client)  # default setting is unallocated

        # should check for some things here, e.g. can we verify a client is allocated or not?
        rospy.logwarn("Scheduler : update from concert clients!")
        self._update()
        self._lock.release()

    ##########################################################################
    # Requester Callback
    ##########################################################################
    # Later this should remove the need for _process_requests above.

    def _requester_update(self, request_set):
        '''
          Callback used in rocon_scheduler_requests scheduler for processing incoming resource requests.
          This gets fired every time an incoming message arrives from a Requester.

          @param request_set : a snapshot of all requests from a single requester in their current state.
          @type rocon_scheduler_requests.transition.RequestSet
        '''
        self._request_set = request_set
        for r in request_set.requests.values():
            print("Request: %s" % r.msg)
        self._lock.acquire()
        rospy.logwarn("Scheduler : update from requester update!")
        self._update()
        self._lock.release()

    def _grant(self, request_id, concert_client):
        '''
          Need to update the resource information with the exact resource provided

          Note : this must be protected by being called inside locked code

          @param request_id : hex string of request id
          @type uuid hex string

          @param concert_client : the concert client that is being allocated to this resource
          @type 
        '''
        for resource_reply in self._request_set.values():
            if request_id == unique_id.toHexString(resource_reply.msg.id):
                rospy.logwarn("Scheduler : grant id matched")

    def _update(self):
        """
          Logic for allocating resources after there has been a state change in either the list of
          available concert clients or the incoming resource requests.

          Note : this must be protected by being called inside locked code
        """
        if self._request_set is None:
            return  # Nothing to do
        rospy.loginfo("Scheduler : updating")
        # get all requests for compatibility tree processing and sort by priority
        # this is a bit inefficient, should just sort the request set directly? modifying it directly may be not right though
        requests = [r.msg for r in self._request_set.requests.values()]
        requests[:] = sorted(requests, key=lambda request: request.priority)
        last_failed_priority = None  # used to check if we should block further allocations to lower priorities
        for request in requests:
            if last_failed_priority is not None and request.priority < last_failed_priority:
                rospy.loginfo("Scheduler : ignoring lower priority requests until higher priorities are filled")
                break
            request_id = unique_id.toHexString(request.id)
            compatibility_tree = impl.create_compatibility_tree(request.resources, [client for client in self._clients.values() if not client.allocated])
            compatibility_tree.print_branches("Compatibility Tree")
            pruned_branches = impl.prune_compatibility_tree(compatibility_tree, verbosity=True)
            pruned_compatibility_tree = impl.CompatibilityTree(pruned_branches)
            pruned_compatibility_tree.print_branches("Pruned Tree", '  ')
            if pruned_compatibility_tree.is_valid():
                rospy.loginfo("Scheduler : allocating")
                last_failed_priority = None
                for branch in pruned_compatibility_tree.branches:
                    for leaf in branch.leaves:  # there should be but one
                        # find the corresponding client
                        leaf.allocate(request_id, branch.limb)
            else:
                last_failed_priority = request.priority
                rospy.loginfo("Scheduler : not ready yet")
