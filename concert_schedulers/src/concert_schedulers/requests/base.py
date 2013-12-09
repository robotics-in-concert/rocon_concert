#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

# local imports

##############################################################################
# Classes
##############################################################################


class RequestVariant(object):
    '''
      Defines a variant used for requests of a higher level nature.
    '''
    __slots__ = [
            '_resources',  # the base requester class from rocon_scheduler_requests
        ]

    def __init__(self):
        self._resources = []

#    def add_resource(self, resource):
##        if 
##        if self._resources:
##            if self._resources[0].name != resource.name || self._resources[0].platform_info != resources.platform_info:
##                
##        self.resources.append()
#        if not self._validate():
#            # @todo raise an exception
#            pass
#
#    def _validate(self):
#        '''
#          Check that the stored resources are all of the same type:
#          i.e. name, platform_info (not parameters or remappings)
#        '''
#        # Check that
#        return True
