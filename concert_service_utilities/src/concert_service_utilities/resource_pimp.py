#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Simple script to pimp out resource for rocon interactions.
#
# - watch the app manager status and when it has a remote controller,
# - flip a spawn/kill pair across
# - call the spawn api
#  - the turtle herder will flip back some handles then.

##############################################################################
# Imports
##############################################################################

import abc
import sys 
import threading
import time

import rospy
import rocon_python_comms
import rocon_uri
import concert_service_utilities
import concert_scheduler_requests
import unique_id
import rocon_std_msgs.msg as rocon_std_msgs
import scheduler_msgs.msg as scheduler_msgs
import concert_service_msgs.msg as concert_service_msgs


class ResourcePimp(object):

    __meta_class = abc.ABCMeta

    '''
    Listens for requests to gain a necessary resource
    '''
    __slots__ = [
        'service_priority',
        'service_id',
        'concert_clients_subscriber',
        'allocation_timeout',
        'avaialble_resource_publisher_name',
        'avaialble_resources',
        'requester',
        'lock',
        'pending_requests',
        'allocated_requests'
        'resource_type'
    ]

    def __init__(self):

        try:
            known_resources_topic_name = rocon_python_comms.find_topic('scheduler_msgs/KnownResources', timeout=rospy.rostime.Duration(5.0), unique=True)
        except rocon_python_comms.NotFoundException as e:
            self.logerr("could not locate the scheduler's known resources topic [%s]" % str(e))
            sys.exit(1)

        self.setup_variables()

        self.lock = threading.Lock()
        self.concert_clients_subscriber = rospy.Subscriber(known_resources_topic_name, scheduler_msgs.KnownResources, self.ros_scheduler_known_resources_callback)
        self.available_resource_publisher = rospy.Publisher(self.available_resource_publisher_name, rocon_std_msgs.StringArray, latch=True, queue_size=1)
        self.available_resources = []
        self.requester = self.setup_requester(self.service_id)
        self.pending_requests = []
        self.allocated_requests = {}
        self.allocate_resource_service_pair_server = rocon_python_comms.ServicePairServer(self.capture_topic_name, self.capture_callback, concert_service_msgs.CaptureResourcePair, use_threads=True)
        self.allocation_timeout = rospy.get_param('allocation_timeout', 15.0)  # seconds

    def setup_requester(self, uuid):
        try:
            scheduler_requests_topic_name = concert_service_utilities.find_scheduler_requests_topic()
            #self.loginfo("Service : found scheduler [%s][%s]" % (topic_name))
        except rocon_python_comms.NotFoundException as e:
            self.logerr("TeleopPimp : %s" % (str(e)))
            return  # raise an exception here?
        frequency = concert_scheduler_requests.common.HEARTBEAT_HZ
        return concert_scheduler_requests.Requester(self.requester_feedback, uuid, 0, scheduler_requests_topic_name, frequency)

    def ros_scheduler_known_resources_callback(self, msg):
        '''
          identify and store changes to the
          available resource list - we get this list via the resource_pool topic
          provided by the scheduler for introspection.

          :param msg: incoming message
          :type msg: scheduler_msgs.KnownResources
        '''
        # find difference of incoming and stored lists based on unique concert names
        diff = lambda l1, l2: [x for x in l1 if x.uri not in [l.uri for l in l2]]
        # get all currently invited teleopable robots
        available_resources = [r for r in msg.resources if self.resource_type in r.rapps and r.status == scheduler_msgs.CurrentStatus.AVAILABLE]
        preemptible_resources = [r for r in msg.resources if self.resource_type in r.rapps and r.status == scheduler_msgs.CurrentStatus.ALLOCATED and r.priority < self.service_priority]
        resources = available_resources + preemptible_resources
        self.lock.acquire()
        new_resources = diff(resources, self.available_resources)
        lost_resources = diff(self.available_resources, resources)
        for resource in new_resources:
            self.available_resources.append(resource)
        for resource in lost_resources:
            # rebuild list in place without lost client
            self.available_resources[:] = [r for r in self.available_resources if resource.uri != r.uri]
        self.lock.release()
        self.publish_available_resources()

    def publish_available_resources(self):
        self.lock.acquire()
        #self.logwarn("Publishing: %s" % [r.status for r in self.teleopable_robots])
        msg = rocon_std_msgs.StringArray()
        msg.strings = [r.uri for r in self.available_resources if r.status != scheduler_msgs.CurrentStatus.ALLOCATED]
        self.available_resource_publisher.publish(msg)
        self.lock.release()

    def requester_feedback(self, request_set):
        '''
          Keep an eye on our pending requests and see if they get allocated here.
          Once they do, kick them out of the pending requests list so _ros_capture_teleop_callback
          can process and reply to the interaction.

          :param request_set: the modified requests
          :type request_set: dic { uuid.UUID : scheduler_msgs.ResourceRequest }
        '''
        for request_id, request in request_set.requests.iteritems():
            #self.logwarn("DJS : request %s has status [%s]" % (request_id, request.msg.status))
            if request.msg.status == scheduler_msgs.Request.GRANTED:
                if request_id in self.pending_requests:
                    self.pending_requests.remove(request_id)
            elif request.msg.status == scheduler_msgs.Request.CLOSED:
                self.pending_requests.remove(request_id)
                self.allocated_requests.remove(request_id)

    def cancel_all_requests(self):
        '''
          Exactly as it says! Used typically when shutting down or when
          it's lost more allocated resources than the minimum required (in which case it
          cancels everything and starts reissuing new requests).
        '''
        #self.lock.acquire()
        self.requester.cancel_all()
        self.requester.send_requests()
        #self.lock.release()

    def capture_callback(self, request_id, msg):
        self.lock.acquire()
        response = self.ros_capture_callback(request_id, msg)
        self.allocate_resource_service_pair_server.reply(request_id, response)
        self.lock.release()


    def send_allocation_request(self, resource):
        # Todo : request the scheduler for this resource,
        # use self.allocation_timeout to fail gracefully
        resource_request_id = self.requester.new_request([resource], priority=self.service_priority)
        self.pending_requests.append(resource_request_id)
        #rospy.logwarn("DJS : resource request id of new request [%s]" % resource_request_id)
        self.requester.send_requests()

        timeout_time = time.time() + self.allocation_timeout
        request_result = False
        while not rospy.is_shutdown() and time.time() < timeout_time:
            if resource_request_id not in self.pending_requests:
                self.allocated_requests[resource.uri] = resource_request_id
                request_result = True
                break
            rospy.rostime.wallsleep(0.1)

        if request_result == False:
            self.requester.rset[resource_request_id].cancel()
            return False, None
        else:
            return True, resource_request_id

    def send_releasing_request(self, uri):
        if uri in self.allocated_requests.keys():
            self.loginfo("released teleopable robot [%s][%s]" % (uri, self.allocated_requests[uri].hex))
            self.requester.rset[self.allocated_requests[uri]].cancel()
            self.requester.send_requests()
            del self.allocated_requests[uri]

    @abc.abstractmethod
    def setup_variables(self):
        pass

    @abc.abstractmethod
    def ros_capture_callback(self, request_id, msg):
        '''
         Processes the service pair server 'capture_resource'. This will run
         in a thread of its own for each request. It has a significantly long lock
         though - this needs to get fixed.
        '''
        pass

    @abc.abstractmethod
    def loginfo(self, msg):
        rospy.loginfo("ResourcePimp : %s"%str(msg))

    @abc.abstractmethod
    def logwarn(self, msg):
        rospy.logwarn("ResourcePimp : %s"%str(msg))

    @abc.abstractmethod
    def logerr(self, msg):
        rospy.logerr("ResourcePimp : %s"%str(msg))
