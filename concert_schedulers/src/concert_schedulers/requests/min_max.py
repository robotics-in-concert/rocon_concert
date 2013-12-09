#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

# local imports
import base
from .exceptions import InvalidRequestException

##############################################################################
# Classes
##############################################################################


class MinMaxRequest(base.RequestVariant):

    __slots__ = [
            '_resources',  # scheduler_msgs.Resources
            '_min',
            '_max',
        ]

    def __init__(self, mininmum, maximum, resources, minimum_priority, maximum_priority):
        '''
          Constructor fully initialises this request.

          @param minimum
          @type int

          @param maximum
          @type int

          @param resources : list of resources to pass to the requester.
          @type scheduler_msgs.Resource[]
        '''
        self._resources = resources
        self._min = min
        self._max = max
        self._validate()  # can raise an exception

    def _validate(self):
        '''
          Check that the stored resources are all of the same type:
          i.e. name, platform_info (not parameters or remappings)
        '''
        if self._min < 0:
            raise InvalidRequestException("Requester : attempted to create invalid min-max request [%s < 0]" % self._min)
        if len(self._resources) < self._min:
            raise InvalidRequestException("Requester : attempted to create invalid min-max request [# resources < minimum]")
        if len(self._resources) > self._max:
            raise InvalidRequestException("Requester : attempted to create invalid min-max request [# resources > maximum]")
        template = self._resources[]
        for resource in self._resources:
            if template.name != resource.name:
                raise InvalidRequestException("Requester : attempted to create invalid min-max request [%s != %s]" % (template.name, resource.name))
            if template.platform_info != resource.platform_info:
                raise InvalidRequestException("Requester : attempted to create invalid min-max request [%s != %s]" % (template.platform_info, resource.platform_info))

    def initial_request(self):
        '''
          Set of resources that would make up an initial request - this needs to be the necessary
          level in order to satisfy the higher level requirements of this type of request (in this
          case the minimum boundary).

          @return list of resources for the request.
          @rtype scheduler_msgs.Resource
        '''
        resources = []
#        for i in range(0:min):
#            resources[] = self._resources[i]
