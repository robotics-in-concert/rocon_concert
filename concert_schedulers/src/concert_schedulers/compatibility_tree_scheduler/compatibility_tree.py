#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
"""
.. module:: compatibility_tree_scheduler.compatibility_tree

This module is the engine behind the resource allocation selection
algorithm.
"""
##############################################################################
# Imports
##############################################################################

import rocon_console.console as console
import concert_msgs.msg as concert_msgs

# local imports

##############################################################################
# Classes
##############################################################################


class CompatibilityBranch(object):
    __slots__ = [
            'limb',    # scheduler_msgs.Resource
            'leaves',  # compatibility_tree_scheduler.ConcertClient
            'name'     # alias to limb.name
        ]

    # This used to work for varying numbers of leaves
    minimum_leaves = 1
    maximum_leaves = 1

    def __init__(self, resource):
        '''
          Forms an empty branch.
          :param scheduler_msgs.Resource resource:
        '''
        self.limb = resource
        self.leaves = []

        # aliases
        self.name = self.limb.rapp

    def prune_leaves(self, leaves):
        '''
          We prune by name, which we know is unique (avoids worrying about python references).

          :param leaves:
          :type leaves: [concert_msgs.ConcertClient]
        '''
        for leaf in leaves:
            self.leaves[:] = [l for l in self.leaves if l.name != leaf.name]

    def redundancy(self):
        '''
          Computes the difference between the number of leaves and the
          minimum required number of leaves.

          :returns: how much we can potentially overallocate.
          :rtype: int
        '''
        return len(self.leaves) - CompatibilityBranch.minimum_leaves

    def __str__(self):
        return console.cyan + self.limb.uri + console.reset + " : " + console.yellow + "%s" % [leaf.name for leaf in self.leaves] + console.reset


def create_compatibility_tree(resources, concert_clients):
    '''
      Checks off implementation node rules and matches them to potential clients fulfilling the
      required conditions (platform tuple, app, optionally name)

      :param resources:
      :type resources:

      :param concert_clients: name indexed dictionary of concert client information
      :type concert_clients: concert_msgs.ConcertClient[]

      :returns: list of tuples, each of which is a node and list of clients that satisfy requirements for that node.
      :rtype: :class:`.CompatibilityTree`
    '''
    compatibility_tree = CompatibilityTree([])
    for resource in resources:
        compatibility_tree.branches.append(CompatibilityBranch(resource))
    for branch in compatibility_tree.branches:
        branch.leaves.extend([client for client in concert_clients if client.is_compatible(branch.limb)])
    return compatibility_tree


def prune_compatibility_tree(compatibility_tree, verbosity=False):
    '''
      Recursive function that checks rule min/max allotments and prunes
      the compatibility tree. It assumes none are currently fixed, so
      pruning can happen anywhere.

      :param compatibility_tree: branches listing compatible resource - clients relationships
      :type compatibility_tree: :class:`.CompatibilityTree`
      :param bool verbosity: adds some pretty printed output to screen for debugging.
      :returns: the pruned branches
      :rtype: [:class:`.CompatibilityBranch`]
    '''
    pruned_branches = []
    if verbosity:
        compatibility_tree.print_branches("Pruning...", "  ")

    ##########################################
    # Stage 1 - prune resolvable branches
    ##########################################
    newly_pruned_branches, remaining_compatibility_tree = _prune_resolvable_branches(compatibility_tree, verbosity)
    pruned_branches.extend(newly_pruned_branches)
    # if there was an update and there is work left to do, dig down
    if newly_pruned_branches:
        if remaining_compatibility_tree is not None:
            pruned_branches.extend(prune_compatibility_tree(remaining_compatibility_tree, verbosity))
        return pruned_branches  # nothing was done

    ##########################################
    # Stage 2 - prune least valuable leaves
    ##########################################
    # Note - there was no newly_pruned_branches here, so we are
    # still dealing with the entire compatibility tree
    pruned_compatibility_tree = _prune_least_valuable_leaf(compatibility_tree, verbosity)
    if pruned_compatibility_tree is not None:
        pruned_branches.extend(prune_compatibility_tree(pruned_compatibility_tree, verbosity))
    else:
        pruned_branches.extend(compatibility_tree.branches)
    return pruned_branches


def _prune_resolvable_branches(compatibility_tree, verbosity):
    '''
      todo.

      :param compatibility_tree: branches listing compatible resource - clients relationships
      :type compatibility_tree: :class:`.CompatibilityTree`
      :param bool verbosity: adds some pretty printed output to screen for debugging.
      :returns: the pruned branches and the trimmed tree
      :rtype: ([:class:`.CompatibilityBranch`], :class:`.CompatibilityTree`)
    '''
    if verbosity:
        #print("")
        compatibility_tree.print_branches("Pruning Resolvable Branches", "    ")
    pruned_branches = []
    remaining_branches = []
    branches = compatibility_tree.branches
    removed_leaves = []
    # look for a branch that can't be worked on any more - i.e. is either
    # empty, or has only one leaf
    for branch in branches:
        if not branch.leaves:
            if not pruned_branches:  # Only accept one change at a time
                pruned_branches.append(branch)
            else:
                remaining_branches.append(branch)
        elif len(branch.leaves) == 1:
            if not pruned_branches:
                pruned_branches.append(branch)
                removed_leaves.extend(branch.leaves)
            else:
                remaining_branches.append(branch)
        else:
            remaining_branches.append(branch)
    removed_leaves = list(set(removed_leaves))  # get a unique list
    for branch in remaining_branches:
        branch.prune_leaves(removed_leaves)
    # are we guaranteed of clearing all of these?
    # if there was an update and there is work left to do, dig down
    if pruned_branches:
        if remaining_branches:
            if verbosity:
                console.pretty_println("      --> pruned leaves: %s" % ' '.join([leaf.name for leaf in removed_leaves]), console.green)
                console.pretty_println("      --> pruned resolvable branches: %s" % ' '.join([branch.name for branch in pruned_branches]), console.green)
            return pruned_branches, CompatibilityTree(branches=remaining_branches)
        else:
            return pruned_branches, None
    else:
        if verbosity:
            console.pretty_println("      --> no resolvable branches", console.green)
        return [], None


def _prune_least_valuable_leaf(compatibility_tree, verbosity):
    '''
       This should be only called on a tree with branches
       that have redundancy, i.e. more potential clients than
       required.

       Does this by scanning the subtree looking for the least
       visible leaf (client) and pruning it from all branches except
       the thinnest branch it exists on (fewest other leaves).
       This branch is typically the most sensitive to variations so we
       attack it first.

       :param compatibility_tree:
       :type compatibility_tree: :class:`.CompatibilityTree`
       :returns: the pruned compatibility_tree:
       :rtype: :class:`.CompatibilityTree`
   '''
    if verbosity:
        compatibility_tree.print_branches('Pruning Least Valuable Leaf', '    ')
    leaf_count = {}  # client name - integer count
    leaves = {}
    thinnest_branch_leaf_count = {}
    # Make a database of all the leaves in all the branches and count how many
    # times a leaf shows up and which branch is it's most sensitive.
    for branch in compatibility_tree.branches:
        for leaf in branch.leaves:
            if leaf.name not in leaves:
                leaves[leaf.name] = leaf
                thinnest_branch_leaf_count[leaf.name] = len(branch.leaves)
                leaf_count[leaf.name] = 1 if not leaf.allocated else 100  # weight in favour of unallocated clients
            else:
                leaf_count[leaf.name] = leaf_count[leaf.name] + 1
            if len(branch.leaves) < thinnest_branch_leaf_count[leaf.name]:
                thinnest_branch_leaf_count[leaf.name] = len(branch.leaves)
    # Now find the least visible leaf, that's the one we have to lock down first.
    least_visible_count = min(leaf_count.values())
    least_visible_leaf_names = [name for name in leaf_count if leaf_count[name] == least_visible_count]
    least_visible_leaf = None
    for leaf in least_visible_leaf_names:
        if least_visible_leaf is None or thinnest_branch_leaf_count[leaf] < thinnest_branch_leaf_count[least_visible_leaf.name]:
            least_visible_leaf = leaves[leaf]
    # Lock down the thinnest branch that leaf shows up on.
    pruned_compatibility_tree = compatibility_tree
    for branch in pruned_compatibility_tree.branches:
        if least_visible_leaf.name in [leaf.name for leaf in branch.leaves]:
            if len(branch.leaves) == thinnest_branch_leaf_count[least_visible_leaf.name]:
                branch.leaves = [least_visible_leaf]
                break
    # And prune it from elsewhere
    for branch in pruned_compatibility_tree.branches:
        if len(branch.leaves) != 1:
            branch.prune_leaves([least_visible_leaf])
    if verbosity:
        console.pretty_println("      --> pruning least visible leaf: %s" % least_visible_leaf.name, console.green)
    return pruned_compatibility_tree


def print_branches(branches, name='Branches', indent=''):
    console.pretty_println(indent + "%s" % name, console.bold)
    for branch in branches:
        print(indent + "  %s" % branch)


def print_leaves(leaves, name='Leaves', indent=''):
    console.pretty_println(indent + "%s" % name, console.bold)
    console.pretty_print(indent + "  Clients: ", console.cyan)
    console.pretty_println("%s " % [leaf.name for leaf in leaves], console.yellow)


class CompatibilityTree(object):
    '''
    Stores the resource - concert client list compatibility
    tree with algorithms to manipulate it.

    Here a branch is a resource, e.g. ``rocon_apps/teleop``, on which leaves
    are the concert clients that can run that resource (i.e. compatible).
    '''
    ERROR_NONE = 0
    ERROR_MINIMUM_REQUIREMENT_UNMET = 1
    ERROR_EXCEEDED_MAXIMUM_REQUIREMENT = 2
    ERROR_DUPLICATE_LEAVES = 3

    def __init__(self, branches):
        """
        Do not use this directly. Use :func:`.create_compatibility_tree` instead.
        """
        self.branches = branches
        """
        The branches on the tree (dict like objects of {resource : clients} type.
        """
        self.error_message = ""  # Used to indicate a failure reason.
        self.error_type = CompatibilityTree.ERROR_NONE

    #def branches(self):
    #    return [branch.limb for branch in self.branches]

    def leaves(self):
        """
        Return a list of all the leaves (concert clients) on the tree.
        :returns: leaves
        :rtype: :class:`.common.ConcertClient`
        """
        leaves = []
        for branch in self.branches:
            leaves.extend(branch.leaves)
        return list(set(leaves))

    def is_valid(self):
        """
        Checks to see if the compatibility tree is a valid tree. Note this has
        nothing to do with allocatability, just checks that minimum leaves are met
        and there are no duplicate leaves.
        :returns: valid or not
        :rtype: bool
        """
        leaves = []
        self.error_message = ""
        for branch in self.branches:
            # Check min, max.
            if len(branch.leaves) < CompatibilityBranch.minimum_leaves:
                self.error_mode = CompatibilityTree.ERROR_MINIMUM_REQUIREMENT_UNMET
                self.error_message = "waiting for clients of type " + branch.name + " [" + str(len(branch.leaves)) + " < " + str(CompatibilityBranch.minimum_leaves) + "]"
                return False
            for leaf in branch.leaves:
                if leaf in leaves:
                    self.error_mode = CompatibilityTree.ERROR_DUPLICATE_LEAVES
                    self.error_message = branch.name + "more than one occurrence of a leaf [" + str(leaf) + "]"
                    return False
                else:
                    leaves.append(leaf)
        return True

    def print_branches(self, name='Branches', indent=''):
        console.pretty_println(indent + "%s" % name, console.bold)
        for branch in self.branches:
            print(indent + "  %s" % branch)

    def add_leaf(self, leaf):
        '''
          Just add to the first compatible branch (worry about more intelligent adding later)

          :param leaf:
          :type leaf: :class:`.common.ConcertClient`.

        '''
        for branch in self.branches:
            if branch.node.is_compatible(leaf):
                if branch.maximum_leaves == concert_msgs.LinkNode.UNLIMITED_RESOURCE or len(branch.leaves) < branch.maximum_leaves:
                    branch.leaves.append(leaf)
                    return branch
        return None

    def remove_leaf(self, leaf):
        '''
          Just remove the first matching leaf name. Assumption is the name is unique of course
          (guaranteed by the conductor).

          :param leaf:
          :type leaf: :class:`.common.ConcertClient`.
        '''
        for branch in self.branches:
            for l in branch.leaves:
                if leaf.name == l.name:
                    branch.leaves[:] = [l for l in branch.leaves if leaf.name != l.name]
                    break
