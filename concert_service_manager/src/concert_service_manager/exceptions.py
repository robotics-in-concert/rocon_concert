#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

##############################################################################
# Exceptions
##############################################################################


class InvalidServiceDescription(Exception):
    pass


class NoConfigurationUpdateException(Exception):
    pass


class NoServiceExistsException(Exception):

    def __init__(self, name):
        self.name = name
    pass


class InvalidSolutionConfigurationException(Exception):
    pass
