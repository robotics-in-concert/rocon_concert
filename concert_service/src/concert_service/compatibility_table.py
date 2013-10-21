#!/usr/bin/env python

import rospy

from concert_msgs.msg import *

class CompatibilityTable(object):

    nodes = []

    def __init__(self,nodes):
        self.nodes = nodes

    def is_ready(self,client_list): 

        # returns status as well as pair of app and client
        return ConcertService.NOT_READY, []
