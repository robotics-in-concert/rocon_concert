#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import uuid
import rospy

from .exceptions import ServiceInfoException

##############################################################################
# Methods
##############################################################################


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
        key = uuid.UUID(rospy.get_param(service_root_namespace + "uuid"))
    except KeyError as e:
        raise ServiceInfoException("Could not find the service parameters [%s]" % e)
    # I don't like returning like this, but keeping it simple for now.
    return (name, description, key)
