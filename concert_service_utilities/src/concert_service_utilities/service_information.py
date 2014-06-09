#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import uuid
import rospy
import rocon_python_comms
import concert_msgs.msg as concert_msgs
import rocon_console.console as rocon_console 
import sys
import os
from .exceptions import ServiceInfoException

# ##############################################################################
# # Methods
# ##############################################################################
class Object(object):
    pass
 
def print_info(name, attribute):
    print(rocon_console.cyan + name + rocon_console.yellow + str(attribute) + rocon_console.reset)
 
def get_services_info():
    '''
      Used to get concert_service_info
       
      resultdict stores key as resource_name and value as tuple of services info
      keyList stores list of resoure_names to be used as key's for resultdict
 
      @return the concert service information as a tuple (keyList, resultdict)
      @rtype (list, dictionary)
    '''
    resultdict = {}
    keyList = []
    try:
        topic_name = rocon_python_comms.find_topic('concert_msgs/Services', timeout=rospy.rostime.Duration(5.0), unique=True)
    except rocon_python_comms.NotFoundException as e:
        print(rocon_console.red + "failed to find unique topic of type 'concert_msgs/Services' [%s]" % str(e) + rocon_console.reset)
        sys.exit(1)
 
    service_info_proxy = rocon_python_comms.SubscriberProxy(topic_name, concert_msgs.Services)
    try:
        service_info_proxy.wait_for_publishers()
    except rospy.exceptions.ROSInterruptException:
        rospy.logwarn('Concert Service Info : ros shut down before concert info could be found.')
 
    trial = 0
    MAX_TRIAL = 5
    while not rospy.is_shutdown():
        result = service_info_proxy(rospy.Duration(0.2))
        if result:
            service_info = result
            break
        rospy.rostime.wallsleep(1.0)  # human time
        trial = trial + 1
        if trial > MAX_TRIAL:
            rospy.logerr('Concert Service info : concert is not found within ' + str(MAX_TRIAL) + ' trials')
            sys.exit(1)
        
    for s in service_info.services:
        objResult = Object()
        objResult.resource_name = s.resource_name
        objResult.name = s.name 
        objResult.description = s.description 
        objResult.author = s.author 
        objResult.priority = s.priority 
        objResult.launcher_type = s.launcher_type 
        objResult.status = s.status 
        objResult.enabled = s.enabled 
        objResult.icon = s.icon 
        resultdict[objResult.resource_name] = (objResult)
        keyList.append(objResult.resource_name)
    return (keyList, resultdict)
 
 
def get_service_info():
    '''
      Used by a service to introspect about itself. This information it is
      retreiving is left like crumbs in a known place by the service manager
      for the individual services to find and retrieve.
 
      Note, this is quite simple and returning bare basics right now. Could
      expand this to a proper ServiceInfo msg later if it gets more complicated.
 
      @return the concert service information as a tuple (name, description, key)
      @rtype (string, string, uuid.UUID)
 
      @raise ServiceInfoNotFoundException : when the information could not be found.
    '''
     
 
    # Namespaces will always be of the form /services/_service_name_/
    # Let's dig down just in case the node we're running is nested deep inside the
    # service namespace and not at its root level.
    namespace = rospy.get_namespace()
    namespace_list = namespace.split('/')
    # if it's of the right form, this list should be ['', 'services', 'service_name']
    if len(namespace_list) < 3:
        raise ServiceInfoException("not called from a concert services ('/services/_name_/') namespace [%s] % namespace")
    if namespace_list[1] != "services":
        raise ServiceInfoException("not called from a concert services ('/services/_name_/') namespace [%s] % namespace")
    name = namespace_list[2]
    service_root_namespace = "/services/" + name + "/"
    #service_info = concert_msgs.ConcertInfo()
    try:
        name_parameter = rospy.get_param(service_root_namespace + "name")
        if name_parameter != name:
            raise ServiceInfoException("service namespace and service name parameter do not match [%s][%s]" % (name, name_parameter))
        description = rospy.get_param(service_root_namespace + "description")
        priority = int(rospy.get_param(service_root_namespace + "priority"))
        key = uuid.UUID(rospy.get_param(service_root_namespace + "uuid"))
    except KeyError as e:
        raise ServiceInfoException("Could not find the service parameters [%s]" % e)
    # I don't like returning like this, but keeping it simple for now.
    return (name, description, priority, key)
 
 
 
# ##############################################################################
# # Main
# ##############################################################################
 
def console_only_main(node_name='concert_service_info', title='Concert Service Information'):
    rospy.init_node(node_name)
    
    try:
        topic_name = rocon_python_comms.find_topic('concert_msgs/Services', timeout=rospy.rostime.Duration(5.0), unique=True)
    except rocon_python_comms.NotFoundException as e:
        print(rocon_console.red + "failed to find unique topic of type 'concert_msgs/Services' [%s]" % str(e) + rocon_console.reset)
        sys.exit(1)
  
    service_info_proxy = rocon_python_comms.SubscriberProxy(topic_name, concert_msgs.Services)
    try:
        service_info_proxy.wait_for_publishers()
    except rospy.exceptions.ROSInterruptException:
        rospy.logwarn('Concert Service Info : ros shut down before concert info could be found.')
  
    trial = 0
    MAX_TRIAL = 5
    while not rospy.is_shutdown():
        result = service_info_proxy(rospy.Duration(0.2))
        if result:
            service_info = result
            break
        rospy.rostime.wallsleep(1.0)  # human time
        trial = trial + 1
  
        if trial > MAX_TRIAL:
            rospy.logerr('Concert Service info : concert is not found within ' + str(MAX_TRIAL) + ' trials')
            sys.exit(1)
  
    rocon_console.pretty_println('Concert Service Information', rocon_console.bold)
    for s in service_info.services:
        print_info('  Resource      : ', s.resource_name)
        print_info('  Name          : ', s.name)
        print_info('  Description   : ', s.description)
        print_info('  Author        : ', s.author)
        print_info('  Priority      : ', s.priority)
        print_info('  Launcher Type : ', s.launcher_type)
        print_info('  Status        : ', s.status)
        print_info('  Enabled       : ', s.enabled)
        print ''


def main(node_name='concert_service_info', title='Concert Service Information', console=True):
    display_available = True if 'DISPLAY' in os.environ.keys() else False
    try:
        from rqt_gui.main import Main
        qt_available = True
    except ImportError:
        sys.exit("Dead dude")
        qt_available = False
        if display_available and not console:
            print(rocon_console.red + "WARNING: rqt plugin not found, console output only (hint: install concert_qt_service_info)." + rocon_console.reset)
    if console or not display_available or not qt_available:
        console_only_main(node_name, title)
    else:
        main = Main()
        sys.exit(main.main(argv=sys.argv, standalone='concert_service_info'))

