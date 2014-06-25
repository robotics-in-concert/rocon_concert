#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: exceptions

This module defines exceptions raised by the concert_schedulers package.
"""

##############################################################################
# Exceptions
##############################################################################


class FailedToStartRappsException(Exception):
    """
    Internally used to indicate when a rapp failed to start.
    """
    pass


class FailedToAllocateException(Exception):
    """
    A more general catchall for allocation failure.
    """
    pass


class InvalidResourceGroupException(Exception):
    """
    Used by the resource group requester to identify invalid resource group specifications.
    """
    pass
