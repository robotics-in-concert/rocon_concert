#!/usr/bin/env python

##############################################################################
# Imports
##############################################################################

import sys
import copy
import unittest
import concert_orchestra
import rosunit
import concert_msgs.msg as concert_msgs
import rocon_app_manager_msgs.msg as rocon_app_manager_msgs
import rocon_utilities.console as console

##############################################################################
# Apps
##############################################################################

app_listener = rocon_app_manager_msgs.AppDescription(
                   name='rocon_apps/listener',
                   display='Listener',
                   description='Default ros style listener tutorial',
                   platform='linux.ros.*'
                   )
app_talker = rocon_app_manager_msgs.AppDescription(
                   name='rocon_apps/talker',
                   display='Talker',
                   description='Default ros style talker tutorial',
                   platform='linux.ros.*'
                   )
apps = [app_listener, app_talker]

##############################################################################
# Concert Clients
##############################################################################

concert_clients = {}
concert_clients['dude_foo'] = concert_msgs.ConcertClient(name='dude_foo', gateway_name='gateway_one', platform='linux', system='ros', robot='pc', apps=apps)
concert_clients['dude_bar'] = concert_msgs.ConcertClient(name='dude_bar', gateway_name='gateway_two', platform='linux', system='ros', robot='pc', apps=apps)
concert_clients['dudette'] = concert_msgs.ConcertClient(name='dudette', gateway_name='gateway_three', platform='linux', system='ros', robot='pc', apps=apps)

##############################################################################
# Setups
##############################################################################

def setup_chatter_configuration():
    #global app_listener, app_talker, apps, concert_clients
    nodes = []
    nodes.append(concert_orchestra.Node({ 'id': 'dude', 'tuple': 'linux.ros.pc.rocon_apps/listener', 'min': 0, 'max': 2, 'force_name_matching': False}))
    nodes.append(concert_orchestra.Node({ 'id': 'dudette', 'tuple': 'linux.ros.pc.rocon_apps/talker', 'min': 1, 'max': 1, 'force_name_matching': True}))
    return nodes, concert_clients

def setup_incomplete_configuration():
    #global app_listener, app_talker, apps, concert_clients
    nodes = []
    nodes.append(concert_orchestra.Node({ 'id': 'dude', 'tuple': 'linux.ros.pc.rocon_apps/listener', 'min': 0, 'max': 2, 'force_name_matching': False}))
    nodes.append(concert_orchestra.Node({ 'id': 'dudette', 'tuple': 'linux.ros.pc.rocon_apps/talker', 'min': 1, 'max': 1, 'force_name_matching': True}))
    nodes.append(concert_orchestra.Node({ 'id': 'dudley', 'tuple': 'linux.ros.pc.rocon_apps/talker', 'min': 1, 'max': 1, 'force_name_matching': True}))
    return nodes, concert_clients

def setup_unlimited_configuration():
    nodes = []
    nodes.append(concert_orchestra.Node({ 'id': 'dude', 'tuple': 'linux.ros.pc.rocon_apps/listener', 'min': 0, 'max': concert_msgs.LinkNode.UNLIMITED_RESOURCE, 'force_name_matching': False}))
    nodes.append(concert_orchestra.Node({ 'id': 'dudette', 'tuple': 'linux.ros.pc.rocon_apps/talker', 'min': 1, 'max': 1, 'force_name_matching': True}))
    return nodes, concert_clients

##############################################################################
# UnitTestClass
##############################################################################

class TestCompatibles(unittest.TestCase):

    def setUp(self):
        pass

    def test_incomplete_configuration(self):
        console.pretty_println("\n************ Incomplete Configuration ************\n", console.bold)
        nodes, concert_clients = setup_incomplete_configuration()
        compatibility_tree = concert_orchestra.create_compatibility_tree(nodes, concert_clients)
        compatibility_tree.print_branches("\nCompatibility Tree\n", '')
        print("")
        pruned_branches = concert_orchestra.prune_compatibility_tree(compatibility_tree, verbosity=True)
        concert_orchestra.print_branches(pruned_branches, "\nPruned Compatibility Tree\n", '  ')
        for branch in pruned_branches:
            if branch.name() == 'dudette':
               self.assertEquals(1, len(branch.leaves))
               self.assertEquals('dudette', branch.leaves[0].name)
            elif branch.name() == 'dude':
               self.assertEquals(2, len(branch.leaves))
            elif branch.name() == 'dudley':
               self.assertEquals(0, len(branch.leaves))

    def test_chatter_configuration(self):
        console.pretty_println("\n*************** Chatter Configuration ************\n", console.bold)
        nodes, concert_clients = setup_chatter_configuration()
        compatibility_tree = concert_orchestra.create_compatibility_tree(nodes, concert_clients)
        compatibility_tree.print_branches("\nCompatibility Tree\n", '')
        print("")
        pruned_branches = concert_orchestra.prune_compatibility_tree(compatibility_tree, verbosity=True)
        concert_orchestra.print_branches(pruned_branches, "\nPruned Compatibility Tree\n", '  ')
        for branch in pruned_branches:
            if branch.name() == 'dudette':
               self.assertEquals(1, len(branch.leaves))
               self.assertEquals('dudette', branch.leaves[0].name)
            elif branch.name() == 'dude':
               self.assertEquals(2, len(branch.leaves))


    def test_unlimited_configuration(self):
        console.pretty_println("\n************* Unlimited Configuration ************\n", console.bold)
        nodes, concert_clients = setup_unlimited_configuration()
        compatibility_tree = concert_orchestra.create_compatibility_tree(nodes, concert_clients)
        compatibility_tree.print_branches("\nCompatibility Tree\n", '')
        print("")
        pruned_branches = concert_orchestra.prune_compatibility_tree(compatibility_tree, verbosity=True)
        concert_orchestra.print_branches(pruned_branches, "\nPruned Compatibility Tree\n", '  ')
        for branch in pruned_branches:
            if branch.name() == 'dudette':
               self.assertEquals(1, len(branch.leaves))
               self.assertEquals('dudette', branch.leaves[0].name)
            elif branch.name() == 'dude':
               self.assertEquals(2, len(branch.leaves))

def tearDown(self):
        pass

NAME = 'test_compatibles'
if __name__ == '__main__':
    rosunit.unitrun('test_compatibles', NAME, TestCompatibles, sys.argv, coverage_packages=['concert_orchestra'])
