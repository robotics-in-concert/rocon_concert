#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/hydro-devel/concert_orchestra/LICENSE
#
##############################################################################
# Imports
##############################################################################

import copy
import rocon_utilities.console as console
import concert_msgs.msg as concert_msgs

# local imports

##############################################################################
# Classes
##############################################################################


class CompatibilityBranch(object):
    '''
      Forms a relationship between the node and the clients it is related with.
      (node is the branch, clients are the leaves).
    '''
    def __init__(self, node):
        '''
          Forms an empty branch.
          @param node
          @type node.Node
        '''
        self.node = node       # [node.Node]
        self.client_list = []  # [concert_msgs.ConcertClient]
        # aliases
        self.minimum_leaves = self.node.min
        self.maximum_leaves = self.node.max
        self.leaves = self.client_list

    def name(self):
        return self.node.id

    def prune_leaves(self, leaves):
        '''
          We prune by name, which we know is unique (avoids worrying about python references).

          @param leaves
          @type [concert_msgs.ConcertClient]
        '''
        for leaf in leaves:
            self.leaves[:] = [l for l in self.leaves if l.name != leaf.name]

    def redundancy(self):
        '''
          Computes the difference between the number of leaves and the
          minimum required number of leaves.
        '''
        return len(self.leaves) - self.minimum_leaves

    def free_slots(self):
        '''
          Computes the difference between the maximum allowed number of leaves
          and the number of leaves.
        '''
        if self.maximum_leaves == concert_msgs.LinkNode.UNLIMITED_RESOURCE:
            return 3
        else:
            return self.maximum_leaves - len(self.leaves)

    def __str__(self):
        return console.cyan + self.node.id + console.reset + " : " + console.yellow + "%s" % [client.name for client in self.client_list] + console.reset


def create_compatibility_tree(implementation_nodes, concert_clients):
    '''
      Checks off implementation node rules and matches them to potential clients fulfilling the
      required conditions (platform tuple, app, optionally name)

      @param implementation node rules
      @type [node.Node]

      @param concert_clients: name indexed dictionary of concert client information
      @type dic(str, concert_msgs.ConcertClient)

      @return list of tuples, each of which is a node and list of clients that satisfy requirements for that node.
      @rtype CompatibilityTree
    '''
    compatibility_tree = CompatibilityTree([])
    for node in implementation_nodes:
        compatibility_tree.branches.append(CompatibilityBranch(node))
    clients = copy.deepcopy(concert_clients.values())
    for branch in compatibility_tree.branches:
        branch.client_list.extend([client for client in clients if branch.node.is_compatible(client)])
    return compatibility_tree


def prune_compatibility_tree(compatibility_tree, verbosity=False):
    '''
      Recursive function that checks rule min/max allotments and prunes
      the compatibility tree. It assumes none are currently fixed, so
      pruning can happen anywhere.

      @param branches listing compatible node - clients relationships
      @type [ CompatibilityBranch ]
    '''
    pruned_branches = []
    if verbosity:
        compatibility_tree.print_branches("Pruning...", "  ")

    ##########################################
    # Stage 1 - prune resolvable branches
    ##########################################
    newly_pruned_branches, remaining_compatibility_tree = prune_resolvable_branches(compatibility_tree, verbosity)
    pruned_branches.extend(newly_pruned_branches)
    # if there was an update and there is work left to do, dig down
    if newly_pruned_branches:
        if remaining_compatibility_tree is not None:
            pruned_branches.extend(prune_compatibility_tree(remaining_compatibility_tree, verbosity))
        return pruned_branches  # nothing was done

    ##########################################
    # Stage 2 - prune least valuable leaves
    ##########################################
    # Don't change the branches configuration, just trim a leaf
    # back until it only remains on one branch.
    # Use a heuristic here to determine the least valuable branch,
    # i.e. the one that has the largest buffer until it is maxxed out.
    pruned_compatibility_tree = prune_least_valuable_leaf(compatibility_tree, verbosity)
    if pruned_compatibility_tree is not None:
        pruned_branches.extend(prune_compatibility_tree(pruned_compatibility_tree))
    else:
        pruned_branches.extend(compatibility_tree.branches)
    return pruned_branches


def prune_resolvable_branches(compatibility_tree, verbosity):
    if verbosity:
        print("")
        compatibility_tree.print_branches("Pruning Resolvable Branches", "    ")
    pruned_branches = []
    remaining_branches = []
    branches = compatibility_tree.branches
    removed_leaves = []
    for branch in branches:
        if not branch.leaves:
            pruned_branches.append(copy.deepcopy(branch))
        elif len(branch.leaves) <= branch.minimum_leaves:
            pruned_branches.append(copy.deepcopy(branch))
            removed_leaves.extend(branch.leaves)
        else:
            remaining_branches.append(copy.deepcopy(branch))
    removed_leaves = list(set(removed_leaves))
    for branch in remaining_branches:
        branch.prune_leaves(removed_leaves)
    # are we guaranteed of clearing all of these?
    # if there was an update and there is work left to do, dig down
    if pruned_branches:
        if remaining_branches:
            if verbosity:
                console.pretty_println("      --> pruning leaves: %s" % ' '.join([leaf.name for leaf in removed_leaves]), console.green)
                console.pretty_println("      --> pruned automatically resolvable branches: %s\n" % ' '.join([branch.name() for branch in pruned_branches]), console.green)
            return pruned_branches, CompatibilityTree(branches=remaining_branches)
        else:
            return pruned_branches, None
    else:
        if verbosity:
            console.pretty_println("      --> no resolvable branches: \n", console.green)
        return [], None


def prune_least_valuable_leaf(compatibility_tree, verbosity):
    '''
       This should be only called on a tree with branches
       that have redundancy (i.e. more potential clients than
       required (specified by node.min).

       Does this by scanning the subtree looking for the least
       visible leaf (client) not including those with visibility one.

       It also grabs the branches that leaf is on, and there should multiple
       possible branches, and subsequently chooses to lock it down on
       the branch with the least redundancy, i.e. pruning it from
       the rest of the tree.

       This makes sure that there are plenty of possible options to
       cover the branches that match this leaf but are not chosen.

       @param subtree
       @type [CompatibilityBranch]
   '''
    if verbosity:
        compatibility_tree.print_branches('Pruning Least Valuable Leaf', '    ')
    leaf_count = {}  # client name - integer count
    leaves = {}
    for branch in compatibility_tree.branches:
        for leaf in branch.leaves:
            if leaf.name not in leaves:
                leaves[leaf.name] = leaf
            leaf_count[leaf.name] = leaf_count[leaf.name] + 1 if leaf.name in leaf_count else 1
    # remove any with count 1.
    for k, v in leaf_count.items():
        if v == 1:
            del leaf_count[k]
    if not leaf_count:
        if verbosity:
            console.pretty_println("      --> no pruning left to do, unwinding", console.green)
        # Nothing more to do
        return None
    least_visible_count = min(leaf_count.values())
    least_visible_leaf_names = [name for name in leaf_count if leaf_count[name] == least_visible_count]
    # let's just take the first one...should I error check here?
    least_visible_leaf = leaves[least_visible_leaf_names[0]]

    # branches associated with least_visible_leaf
    least_visible_branches = []
    for branch in compatibility_tree.branches:
        if least_visible_leaf in branch.leaves:
            least_visible_branches.append(copy.deepcopy(branch))

    # find most valuable branch - using a complicated formula here:
    # that lets it strip itself from the branches that are close to pushing their max.
    value = lambda b: (1.0/b.redundancy())*min([3, b.free_slots()])
    most_valuable_branch = None
    for branch in least_visible_branches:
        if most_valuable_branch is None or value(branch) > value(most_valuable_branch):
            most_valuable_branch = branch

    pruned_compatibility_tree = copy.deepcopy(compatibility_tree)
    for branch in pruned_compatibility_tree.branches:
        if branch.name() != most_valuable_branch.name():
            branch.prune_leaves([least_visible_leaf])
    return pruned_compatibility_tree


def print_branches(branches, name='Branches', indent=''):
    console.pretty_println(indent + "%s" % name, console.bold)
    for branch in branches:
        print(indent + "  %s" % branch)


def print_leaves(leaves, name='Leaves', indent=''):
    '''
      Pretty prints a list of clients (names only)

      @param leaves
      @type [concert_msgs.ConcertClient]
    '''
    console.pretty_println(indent + "%s" % name, console.bold)
    console.pretty_print(indent + "  Clients: ", console.cyan)
    console.pretty_println("%s " % [leaf.name for leaf in leaves], console.yellow)


class CompatibilityTree(object):
    '''
      Stores the implementation node - concert client list compatibility
      tree with algorithms to manipulate it.
    '''
    def __init__(self, branches):
        self.branches = branches

    def nodes(self):
        return [branch.node for branch in self.branches]

    def leaves(self):
        leaves = []
        for branch in self.branches:
            leaves.extend(branch.leaves)
        return list(set(leaves))
    
    def is_valid(self):
        leaves = []
        for branch in self.branches:
            # Check min, max.
            if len(branch.leaves) < branch.minimum_leaves:
                return False
            if  branch.maximum_leaves != concert_msgs.LinkNode.UNLIMITED_RESOURCE and len(branch.leaves) > branch.maximum_leaves:
                return False
            # Check for more than 1 occurrence of a leaf
            for leaf in branch.leaves:
                if leaf in leaves:
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
        '''
        for branch in self.branches:
            if branch.node.is_compatible(leaf):
                if branch.maximum_leaves == concert_msgs.LinkNode.UNLIMITED_RESOURCE or len(branch.leaves) < branch.maximum_leaves:
                    branch.leaves.append(leaf)
                    return branch
        return None

