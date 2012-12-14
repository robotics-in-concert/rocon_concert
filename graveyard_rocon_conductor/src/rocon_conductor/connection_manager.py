'''
Created on 09/08/2011

@author: snorri
'''
##############################################################################
# Imports
##############################################################################

import os
import xmlrpclib
import threading
import socket #socket.error
import sys

# Ros imports
import roslib; roslib.load_manifest('rocon_conductor')
import rospy

# Rocon imports
import zeroconf_avahi
import concert_msgs
from zeroconf_msgs.msg import *
from zeroconf_msgs.srv import *
from concert_msgs.msg import *

# Local imports
import zeroconf
from .utilities import platform_id_to_string
from .utilities import system_id_to_string

##############################################################################
# Class
##############################################################################


class ConcertClientError(Exception):
    '''
       Input value should be a string
    '''
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return self.value


class ConcertClient(object):
    '''
      Initialises the client with zeroconf information and a few extra
      details from xmlrpc handshaking. These clients are actually created and info stored,
      even if they do not actually connect to the concert. I may have to revert on this decision later.

      @param master zconf_info : zero-configuration to store.

      @var zconf_info : the zero-configuration for this client [zeroconf_msgs.msg.DiscoveredService]
      @var connected  : connection status
      @var app_manager_uri : address and port number of the app manager's xmlrpc handshaking server.
    '''
    def __init__(self, unique_name_generator, zconf_info=None):
        '''
          Can be initialised via zeroconf identification, or we get a connection out of the blue,
          in which case we have no zeroconf information.
        '''
        # Description variables
        self.zconf_info = None
        # System triple
        self.platform_info = concert_msgs.msg.PlatformInfo() 
        self.app_manager_uri = None

        # Status variables
        self.is_connected = False
        self.last_connection_timestamp = None

        # Functionality variables
        self.platform_info_subscriber = None
         
        if zconf_info is not None:
            self.zconf_info = zconf_info
            rospy.logwarn("Conductor: update this to handle multiple addresses - Todo!!")
            # Need to better check for number of addresses here
            self.app_manager_uri = "http://" + self.zconf_info.ipv4_addresses[0] + ":" + str(self.zconf_info.port)
            s = self.configuration_server()
            # get system-platform-robot triple + key (random hex string) + suggested name
            rospy.loginfo("Conductor: requesting information from the app manager at %s"%self.app_manager_uri)
            try:
                self.platform_info.platform = s.platform()
                self.platform_info.system = s.system()
                self.platform_info.robot = s.robot()
                self.platform_info.key = s.key()
                self.platform_info.suggested_name = s.suggested_name()
                # configure this client with a uniquely generated name
                self.platform_info.unique_name = unique_name_generator(self.platform_info.suggested_name)
            except xmlrpclib.Fault, err:
                raise ConcertClientError("could not retrieve platform information [%s][%s][%s]"%(err.faultCode,err.faultString,zconf_info.name))
            except xmlrpclib.ProtocolError:
                raise ConcertClientError("could not contact client on the zeroconf url [%s][%s]"%(self.app_manager_uri,zconf_info.name))
            except IOError, err: # socket.error
                raise ConcertClientError("io error connecting to client [%s][%s][%s]"%(err.errno,os.strerror(err.errno),zconf_info.name))
            except:
                raise ConcertClientError("unexpected error contacting the client's xmlrpc server [%s]"%zconf_info.name)
        
    def to_msg_format(self):
        client = concert_msgs.msg.ConcertClient()
        client.zeroconf = self.zconf_info if (self.zconf_info is not None) else zeroconf_msgs.msg.DiscoveredService() 
        client.platform = platform_id_to_string(self.platform_info.platform) 
        client.system = system_id_to_string(self.platform_info.system) 
        client.robot = self.platform_info.robot if (self.platform_info.robot is not None) else "unknown" 
        client.suggested_name = self.platform_info.suggested_name if (self.platform_info.suggested_name is not None) else "unknown" 
        client.unique_name = self.platform_info.unique_name if (self.platform_info.unique_name is not None) else "unknown" 
        client.key = self.platform_info.key if (self.platform_info.key is not None) else "none" 
        client.app_manager_uri = self.app_manager_uri if (self.app_manager_uri is not None) else "unknown"
        client.is_connected = self.is_connected
        client.last_connection_timestamp =  self.last_connection_timestamp if (self.last_connection_timestamp is not None) else rospy.Time() # 0sec, 0nsec
        return client

    def register_connection(self):
        '''
          Used by the connections manager to save a timestamp highlighting
          when the last client connection was registered. 
        '''
        rospy.loginfo("Conductor: timestamping new connection [%s]"%self.platform_info.unique_name)
        self.last_connection_timestamp = rospy.Time.now()
        self.is_connected = True 
        
    def has_platform_information(self):
        '''
          Part of the android mess where we couldn't query xmlrpc server for 
          platform information - this lets us get it via ros publisher
          after the client is already connected.
        '''
        if self.platform_info_subscriber is not None:
            # We're currently waiting for the callback to kick in
            return False
        elif self.platform_info.platform is None or self.platform_info.system is None:
            return False
        else:
            return True
            
#    def retrieve_platform_information(self, unique_name):    
#        '''
#          Part of the android mess where we couldn't query xmlrpc server for 
#          platform information - this lets us get it via ros publisher
#          after the client is already connected.
#        '''
#        self.platform_info.unique_name = unique_name
#        if self.platform_info_subscriber is not None:
#            rospy.logerr("Conductor: client's platform info publisher was already started (shouldn't be).")
#            return
#        self.platform_info_subscriber = rospy.Subscriber( 
#                        roslib.names.make_global_ns(roslib.names.ns_join(unique_name,"platform_info")), 
#                        concert_msgs.msg.PlatformInfo,
#                        self.retrieve_platform_information_callback) 
#
#    def retrieve_platform_information_callback(self, data):
#        '''
#           This will usually fire immediately once the publisher is initiated as its latched.
#        '''
#        self.platform_info = data
#
#        self.platform_info_subscriber.unregister()
#        del self.platform_info_subscriber
#        self.platform_info_subscriber = None
#        # this is naive, move to the spin() loop to make sure we check all clients for connectivity.
#        self.register_connection()
#        rospy.loginfo("Conductor: client connected [%s][%s][%s]"%(self.unique_name,self.platform,self.system))
        
    def configuration_server(self):                
        return xmlrpclib.ServerProxy(self.app_manager_uri)
    
    def invite_to_concert(self, concert_master_name):
        '''
          Sends an invite to the client (contact's the client's xmlrpc invitation server).
          
          @param concert_master_name : name of the concert master it is being invited to [str]
          @return bool : success or failure
          
          Assumptions: 
            1) for now we're always inviting, and it always works.
        '''
        rospy.loginfo("Conductor: inviting client [%s][%s][%s]"%(self.platform_info.unique_name, self.zconf_info.name, self.app_manager_uri))
        s = self.configuration_server()
        # Set the unique name for the app manager to pair with this concert master
        s.unique_name(concert_master_name, self.platform_info.unique_name)
        try:
            if s.invite(concert_master_name):
                rospy.loginfo("Conductor: client accepted the invitation [%s][%s]"%(self.platform_info.unique_name, self.app_manager_uri))
            else:
                rospy.loginfo("Conductor: client rejected the invitation [%s][%s]"%(self.platform_info.unique_name, self.app_manager_uri))
        except xmlrpclib.Fault, err:
            rospy.logwarn("Conductor: a fault occurred")
            rospy.logwarn("Conductor: fault code: %d" % err.faultCode)
            rospy.logwarn("Conductor: fault string: %s" % err.faultString)
            rospy.logwarn("Conductor: client does not support being invited [%s]"%self.platform_info.unique_name)
        except socket.timeout, err:
            rospy.logwarn("Conductor: socket timed out when trying to invite client [%s][%s][%s]"%(err.errno,os.strerror(err.errno),self.platform_info.unique_name))

class Connections(threading.Thread):
    
    unique_id_counter = 0

    '''
    @var concert_master_name : zeroconf name for the concert master [str]
    @var concert_clients : list of concert clients [ConcertClient[]]
    @var auto_connect_clients : whether to auto-invite clients or not
    
    @param auto_invite_clients
    '''
    def __init__(self):
        '''
           Subscriber callback that listens to the zeroconf new_connection topic.
        '''
        threading.Thread.__init__(self)        
        self.concert_clients = set([])
        self.concert_master_name = zeroconf.concert_master_name()
        rospy.loginfo("Conductor: initialised for concert '%s'"%self.concert_master_name)
        # This is stub functionality, we'll expand auto-invitations properly later
        self._auto_invite_clients = rospy.get_param("auto_invite_clients", True)

        # Ros comms
        self.concert_clients_publisher = rospy.Publisher("concert_clients", concert_msgs.msg.ConcertClients, latch=True)
        rospy.Subscriber("new_connections",zeroconf_msgs.msg.DiscoveredService,self.new_connection_cb)
        rospy.Subscriber("lost_connections",zeroconf_msgs.msg.DiscoveredService,self.lost_connection_cb)
        
        for zconf_client in zeroconf.discover_concert_clients():
            try:
                concert_client = ConcertClient(self.unique_name_generator, zconf_client)
                # should check that its not already in in the concert_clients list
                if self._auto_invite_clients:
                    concert_client.invite_to_concert(self.concert_master_name)
                self.concert_clients.add( concert_client ) # add it whether the invite succeeded or not, details of the success are in the client class
            except ConcertClientError as e:
                rospy.logerr("Conductor: %s"%e)
                rospy.logerr("Conductor: not appending to the concert client list")
        self.publish_discovered_clients()
                

    def unique_name_generator(self, suggested_name):
        '''
          Checks if the suggested name is currently unique and if not, stamps it with a # to guarantee 
          uniqueness.
          
          This could be a little bit 'nicer'. The # increments for every robot, not in parallel for every 
          robot with a particular suggested name. i.e. irobi, robosem, irobo_1, robosem_2, robosem_3, 
          irobi_4.
        '''
        existing_client = next((client for client in self.concert_clients if client.platform_info.unique_name == suggested_name), None)
        if existing_client is None:
            return suggested_name
        else:
            unique_name = suggested_name + "_" + str(Connections.unique_id_counter)
            Connections.unique_id_counter += 1
            return unique_name
        
    def new_registration_cb(self, client):
        rospy.loginfo("Conductor: new registration [%s][%s]"%(client.key, client.unique_name))
        
    def new_connection_cb(self,new_zconf_client):
        '''
           Relays removed zeroconf connections from the zeroconf/new_connections topic.
        '''
        duplicated_client = next((client for client in self.concert_clients if client.zconf_info.name == new_zconf_client.name), None)
        if duplicated_client is not None:
            # Not really checking for this, but if zeroconf is not buggy, this is typically true (might be being naive!).
            rospy.loginfo("Conductor: discovered existing client on a new interface [ignoring][%s]"%new_zconf_client.name)
            #rospy.logwarn("Conductor: existing connection: \n%s"%zeroconf_avahi.utilities.service_to_str(duplicated_client.zconf_info))
            #rospy.logwarn("Conductor: new connection: \n%s"%zeroconf_avahi.utilities.service_to_str(new_zconf_client))
            return
        try:
            client = ConcertClient(self.unique_name_generator, new_zconf_client)
            client.invite_to_concert(self.concert_master_name)
            self.concert_clients.add( client )
            self.publish_discovered_clients()
        except ConcertClientError as err:
            rospy.logerr("Conductor: %s"%err)
            rospy.logerr("Conductor: not appending to the concert client list")
            

    def lost_connection_cb(self,lost_zconf_client):
        '''
           Relays removed zeroconf connections from the zeroconf/lost_connections topic.
        '''
        lost_client = next((client for client in self.concert_clients if client.zconf_info.name == lost_zconf_client.name), None)
        if lost_client is not None:
            rospy.loginfo("Conductor: removing client [%s]"%lost_zconf_client.name)
            self.concert_clients.remove(lost_client)
            self.publish_discovered_clients()
        else:
            rospy.logwarn("Conductor: tried to remove a non-attached client")
    
    def watchdog(self, master):
        '''
          This keeps tab on the ros master's xmlrpc api to check for new incoming connections.
          It's a bit naive, but it does the job - at least until we can do proper invites across
          the board (android is on-connect only atm).
        '''
        caller_id = '/script'
        code, msg, val = master.getSystemState(caller_id)
        if code == 1:
            pubs, unused_subs, unused_srvs = val
            # Later shift the search to a service
            # Here find the publishers with form: _unique_name_/platform_info
            for pub, unused_node in pubs:
                if pub.find("platform_info") != -1:
                    unique_name = pub.split('/')[1]  # is of form /ros_robot/platform_info
                    client = next((client for client in self.concert_clients if client.platform_info.unique_name == unique_name), None)
                    
                    # Initially, our android clients werent' getting found by zeroconf, so 
                    # we had to create the ConcertClient and get platform information here
                    if client is None:
                        # client = ConcertClient(self.unique_name_generator)
                        # self.concert_clients.add( client )
                        # Now we just pass here and wait for zeroconf to create it
                        continue
                    # Now we always get our platform info from the xmlrpc servers, so skip this too
                    # if not client.has_platform_information():
                    #    client.retrieve_platform_information(unique_name)
                    if not client.is_connected:
                        client.register_connection()
                        self.publish_discovered_clients()
        else:
            rospy.logerr("Conductor: failed to call the concert master for the system state [%s][%s]."%(code,msg))

    ##############################################################################
    # Come in spinner!
    ##############################################################################

    def run(self):
        '''
          This loop currently runs a watchdog and publishes current client list.
          See the watchdog method for more detailed information.
           
          Todo: fix a race condition when calling concert_clients in this thread.
        '''
        master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        while not rospy.is_shutdown():
            self.watchdog(master)
            rospy.sleep(2.0)

    def spin(self):
        '''
          Convenient label if we're spinning in the main thread (i.e. not using this class's thread)..
        '''
        self.run()
        
    def join(self):
        ''' 
            Wait for the thread to terminate. Note that it doesn't need help - it will terminate
            when ros shuts down.
        '''
        self.join()
    
    ##############################################################################
    # Utilities
    ##############################################################################

    def publish_discovered_clients(self):
        '''
            Provide a list of currently discovered clients. This goes onto a 
            latched publisher, so subscribers will always have the latest
            without the need to poll a service.
        '''
        discovered_concert = concert_msgs.msg.ConcertClients()
        for client in self.concert_clients:
            discovered_concert.clients.append(client.to_msg_format())
        self.concert_clients_publisher.publish(discovered_concert)
