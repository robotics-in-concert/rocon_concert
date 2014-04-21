#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: exceptions

This module defines exceptions raised by the concert_conductor package.
"""

##############################################################################
# Exceptions
##############################################################################


class ConductorFailureException(Exception):
    """Catostrophic failure in the conductor node"""
    pass


class InvalidTransitionException(Exception):
    """This transition is not permitted"""
    pass
