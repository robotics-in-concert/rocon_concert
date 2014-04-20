#!/usr/bin/env python
#
# License: BSD
#
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import sys
import copy
import unittest
import concert_schedulers
import concert_schedulers.common
import concert_schedulers.compatibility_tree_scheduler as compatibility_tree_scheduler
import rosunit
import concert_msgs.msg as concert_msgs
import scheduler_msgs.msg as scheduler_msgs
import rocon_app_manager_msgs.msg as rocon_app_manager_msgs
import rocon_console.console as console

##############################################################################
# Rapps
##############################################################################

app_listener = rocon_app_manager_msgs.Rapp(
                   name='rocon_apps/listener',
                   display_name='Listener',
                   description='Default ros style listener tutorial',
                   platform='linux.*.ros.*'
                   )
app_talker = rocon_app_manager_msgs.Rapp(
                   name='rocon_apps/talker',
                   display_name='Talker',
                   description='Default ros style talker tutorial',
                   platform='linux.*.ros.*'
                   )
apps = [app_listener, app_talker]

##############################################################################
# Concert Clients
##############################################################################

concert_client_msgs = {}
concert_client_msgs['dude_foo'] = concert_msgs.ConcertClient(name='dude_foo', gateway_name='gateway_one', platform_info='linux.*.ros.pc.*', apps=apps)
concert_client_msgs['dude_bar'] = concert_msgs.ConcertClient(name='dude_bar', gateway_name='gateway_two', platform_info='linux.*.ros.pc.*', apps=apps)
concert_client_msgs['dudette'] = concert_msgs.ConcertClient(name='dudette', gateway_name='gateway_three', platform_info='linux.*.ros.pc.dudette', apps=apps)

concert_clients = {}
for name, msg in concert_client_msgs.iteritems():
    concert_clients[name] = concert_schedulers.common.ConcertClient(msg)

resources = {}
resources['dude']    = scheduler_msgs.Resource(name="rocon_apps/listener",    platform_info="linux.*.ros.pc.*")
resources['dudette'] = scheduler_msgs.Resource(name="rocon_apps/talker", platform_info="linux.*.ros.pc.dudette")

##############################################################################
# Setups
##############################################################################

def setup_chatter_configuration():
    #global app_listener, app_talker, apps, concert_clients
    chatter_resources = []
    chatter_resources.append(resources['dude'])
    chatter_resources.append(resources['dude'])
    chatter_resources.append(resources['dudette'])
    return chatter_resources, concert_clients

##############################################################################
# UnitTestClass
##############################################################################

class TestCompatibilityTreeSchedulers(unittest.TestCase):

    def setUp(self):
        pass

    def test_chatter_configuration(self):
        console.pretty_println("\n*************** Chatter Configuration ************\n", console.bold)
        chatter_resources, chatter_clients = setup_chatter_configuration()
        compatibility_tree = compatibility_tree_scheduler.create_compatibility_tree(chatter_resources, chatter_clients.values())
        compatibility_tree.print_branches("\nCompatibility Tree\n", '')
        print("")
        pruned_branches = compatibility_tree_scheduler.prune_compatibility_tree(compatibility_tree, verbosity=True)
        compatibility_tree_scheduler.print_branches(pruned_branches, "\nPruned Compatibility Tree\n", '  ')
        for branch in pruned_branches:
            if branch.name == 'dudette':
               self.assertEquals(1, len(branch.leaves))
               self.assertEquals('dudette', branch.leaves[0].name)
            elif branch.name == 'dude':
               self.assertEquals(2, len(branch.leaves))

def tearDown(self):
        pass

if __name__ == '__main__':
    rosunit.unitrun('concert_schedulers_compatibility_tree',
                    'test_compatibility_tree_scheduler',
                    TestCompatibilityTreeSchedulers,
                    sys.argv,
                    coverage_packages=['concert_schedulers']
                   )
