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
import rocon_app_manager_msgs.srv as rapp_manager_srvs
import rocon_scheduler_requests

import demo_allocator
from .exceptions import FailedToStartAppsException

##############################################################################
# Classes
##############################################################################


class CompatibilityTreeScheduler(object):

    __slots__ = [
            '_subscribers',
            '_requests',
            '_lock',
            'spin'
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
#        self._clients = {}           # concert_msgs/ConcertClient.name : concert_msgs/ConcertClient of all concert clients
#        self._inuse = {}             #
#        self._pairs = {}             # scheduler_msgs/Resource, concert_msgs/ConcertClient pairs
#        self._shutting_down = False  # Used to protect self._pairs when shutting down.
#        self._setup_ros_api(requests_topic_name)
        self._lock = threading.Lock()
#        self._scheduler = rocon_scheduler_requests.Scheduler(callback=self._requester_update, topic=requests_topic_name)

        # aliases
        self.spin = rospy.spin

    def _setup_ros_api(self, requests_topic_name):
        self._subscribers['concert_client_changes'] = rospy.Subscriber(concert_clients_topic_name, concert_msgs.ConcertClients, self._ros_service_concert_client_changes)
        self._subscribers['requests'] = rospy.Subscriber(requests_topic_name, scheduler_msgs.SchedulerRequests, self._process_requests)

    ##########################################################################
    # Ros callbacks
    ##########################################################################

    def _ros_service_concert_client_changes(self, msg):
        """
            @param
                msg : concert_msgs.ConcertClients

            1. Stops services which client left.
            2. Rebuild client list
            3. Starts services which has all requested resources
        """
        self._lock.acquire()
        clients = msg.clients
        #self._stop_services_of_left_clients(clients)

        self._clients = {}
        self.loginfo("clients:")
        for c in clients:
            self.loginfo("  %s" % c.name)
            self._clients[c.name] = c

        self._update()
        self._lock.release()

    def _process_requests(self, msg):
        """
            Handle incoming requests for resources.

            @param : msg
            @type scheduler_msgs.SchedulerRequests
        """
        self._lock.acquire()
        requester_uuid = unique_id.toHexString(msg.requester)
        self._requests[requester_uuid] = msg.requests
        # This is periodic, so should check if the requests changed before calling update.
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
        pass
        #self.logwarn("requester update callback function!")

    def _update(self):
        """
            Logic for allocating resources after there has been a state change in either the list of
            available concert clients or the incoming resource requests.

            For each request which needs to be started
        """
        rospy.loginfo("Scheduler : updating")
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
