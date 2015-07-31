#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
This is the top-level namespace of the concert_conductor_ ROS
package. It is responsible for inviting and managing concert clients
on the network.

.. _concert_conductor: http://wiki.ros.org/concert_conductor

"""
##############################################################################
# Imports
##############################################################################

# For use by the conductor node script
from .conductor import Conductor
from .concert_client import ConcertClient
from .exceptions import ConductorFailureException

# For state transition graph
from .transitions import StateTransitionTable
