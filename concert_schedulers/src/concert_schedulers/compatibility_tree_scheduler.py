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
            '_requests',
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
        self._requests = {}          # requester uuid : scheduler_msgs/Request[] - keeps track of all current requests
        self._clients = {}           # impl.ConcertClient.name : impl.ConcertClient of all concert clients
        self._lock = threading.Lock()
        self._requests = []

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
        #for r in request_set.requests.values():
        #    print("Request: %s" % r.msg)
        self._lock.acquire()
        self._requests = [r.msg for r in request_set.requests.values()]
        # sort by priority
        self._requests[:] = sorted(self._requests, key=lambda request: request.priority)
        #for request in self._requests:
        #    rospy.loginfo("\nRequest:\n%s" % request)
        #    for resource in request.resources:
        #        print("\n%s" % resource)
        rospy.logwarn("Scheduler : update from requester update!")
        self._update()
        self._lock.release()

    def _update(self):
        """
            Logic for allocating resources after there has been a state change in either the list of
            available concert clients or the incoming resource requests.

            Note : this must be protected by being called inside locked code
        """
        rospy.loginfo("Scheduler : updating")
        last_failed_priority = None  # used to check if we should block further allocations to lower priorities
        for request in self._requests:
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
                    for leaf in branch.leaves:
                        leaf.allocate(request_id, branch.limb)
            else:
                last_failed_priority = request.priority
                rospy.loginfo("Scheduler : not ready yet")
#        for requests in self._requests.values():
#            for request in requests:
#                key = unique_id.toHexString(request.id)
#                if key in self._pairs:
#                    # nothing to touch
#                    continue
#                self.loginfo("processing outstanding request %s" % unique_id.toHexString(request.id))
#                resources = copy.deepcopy(request.resources)
#                app_pairs = []
#                available_clients = self._get_available_clients()
#                cname = [c.name for c in available_clients]
#                self.loginfo("  available clients : " + str(cname))
#                self.loginfo("  required resources : %s" % [resource.platform_info + "." + resource.name for resource in resources])
#
#                # Check if any resource can be paired with already running client
#                paired_resources, reused_client_app_pairs = self._pairs_with_running_clients(resources)
#
#                # More client is required?
#                remaining_resources = [resource for resource in resources if not resource in paired_resources]
#
#                # More client is required?
#                remaining_resources = [r for r in resources if not r in paired_resources]
#
#                # Clients who are not running yet
#                available_clients = self._get_available_clients()
#                cname = [c.name for c in available_clients]
#                self.loginfo("  remaining clients : " + str(cname))
#                self.loginfo("  remaining resources : %s" % [resource.platform_info + "." + resource.name for resource in remaining_resources])
#
#                # Pair between remaining node and free cleints
#                status, message, new_app_pairs = demo_allocator.resolve(remaining_resources, available_clients)
#
#                app_pairs.extend(reused_client_app_pairs)
#                app_pairs.extend(new_app_pairs)
#
#                # Starts request if it is ready
#                if status is concert_msgs.ErrorCodes.SUCCESS:
#                    self._pairs[key] = app_pairs
#                    self._mark_clients_as_inuse(app_pairs)
#                    self._start_request(new_app_pairs, key)
#                else:
#                    # if not, do nothing
#                    self.logwarn("warning in [" + key + "] status. " + str(message))
