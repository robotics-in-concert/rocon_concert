#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

# system

# ros
import rocon_console.console as console

# local
import interactions
from .exceptions import InvalidInteraction

##############################################################################
# Classes
##############################################################################


class InteractionsTable(object):
    '''
      The runtime populated interactions table along with methods to
      manipulate it.
    '''
    __slots__ = [
            'interactions',  # rocon_interactions.interactions.Interaction[]
        ]

    def __init__(self):
        self.interactions = []

    def roles(self):
        return set([i.role for i in self.interactions])

#     def __getitem__(self, role):
#         '''
#           Get an app list for the specified role using the square bracket accessor.
# 
#           @param role
#           @type str
# 
#           @return a list of remocon apps
#           @rtype concert_msgs.RemoconApp[]
#         '''
#         return self.role_and_app_table[role]

#     def __contains__(self, resource_name):
#         return resource_name in self.pool
#
#     def __getitem__(self, name):
#         return self.pool[name]

    def __len__(self):
        return len(self.interactions)

    def __str__(self):
        s = ''
        role_view = self.generate_role_view()
        for role, interactions in role_view.iteritems():
            s += console.bold + role + console.reset + '\n'
            for interaction in interactions:
                s += "\n".join("  " + i for i in str(interaction).splitlines()) + '\n'
        return s

    def generate_role_view(self):
        '''
          Creates a temporary copy of the interactions and sorts them into a dictionary
          view classified by role.

          Note: there's got to be a faster way of doing this.

          @return A role based view of the interactions
          @rtype dict role(str) : interactions.Interaction[]
        '''
        interactions = list(self.interactions)
        role_view = {}
        for interaction in interactions:
            if interaction.role not in role_view.keys():
                role_view[interaction.role] = []
            role_view[interaction.role].append(interaction)
        return role_view

    def load(self, msgs):
        '''
          Load some interactions into the interaction table. This involves some initialisation
          and validation steps.

          @param msgs : a list of interaction specifications to populate the table with.
          @type concert_msgs.Interaction[]

          @return list of all additions and any that were flagged as invalid
          @rtype (interactions.Interaction[], concert_msgs.Interaction[]) : (new, invalid)
        '''
        new = []
        invalid = []
        for msg in msgs:
            try:
                interaction = interactions.Interaction(msg)
                self.interactions.append(interaction)
                self.interactions = list(set(self.interactions))  # uniquify the list, just in case
                new.append(interaction)
            except InvalidInteraction:
                invalid.append(msg)
        return new, invalid

    def unload(self, msgs):
        '''
          Removed the specified interactions interactions table. This list is typically
          the same list as the user might initially send - no hashes yet generated.

          @param msgs : a list of interactions
          @type concert_msgs.Interaction[]

          @return a list of removed interactions
          @rtype concert_msgs.Interaction[]
        '''
        removed = []
        for msg in msgs:
            msg_hash = interactions.generate_hash(msg.name, msg.role, msg.namespace)
            found = next((interaction for interaction in self.interactions if interaction.hash == msg_hash), None)
            if found is not None:
                removed.append(msg)
                self.interactions.remove(found)
        return removed
