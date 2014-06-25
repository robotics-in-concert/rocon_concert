#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
This is the top-level namespace of the concert_schedulers_ ROS
package. It contains various requester-scheduler implementations
and a library of scheduler launchers.

.. _concert_schedulers: http://wiki.ros.org/concert_schedulers

"""
##############################################################################
# Imports
##############################################################################

from .resource_pool_requester.resource_group import ResourcePoolGroup
from .resource_pool_requester.requester import ResourcePoolRequester
from .compatibility_tree_scheduler.scheduler import CompatibilityTreeScheduler
import compatibility_tree_scheduler.compatibility_tree
from .compatibility_tree_scheduler.compatibility_tree import create_compatibility_tree, prune_compatibility_tree
