#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/hydro-devel/concert_orchestra/LICENSE
#
##############################################################################
# Imports
##############################################################################

import re
import concert_msgs.msg as concert_msgs
import rocon_std_msgs.msg as rocon_std_msgs

##############################################################################
# Classes
##############################################################################


class Node(object):
    '''
      This represents an implementation rule node currently derived from
      a ros parameterisation.
    '''
    __slots__ = ['id', 'tuple', 'min', 'max', 'force_name_matching']

    def __init__(self, param):
        # Compulsory parameters - should do some exception checking here
        self.id = param['id']
        self.tuple = param['tuple']
        # Optional parameters
        if not 'min' in param and not 'max' in param:
            self.min = 1
            self.max = 1
        elif not 'min' in param:
            self.min = 1
            self.max = param['max']
        elif not 'max' in param:
            self.min = param['min']
            self.max = concert_msgs.LinkNode.UNLIMITED_RESOURCE
        else:
            self.min = param['min']
            self.max = param['max']
        self.force_name_matching = True if 'force_name_matching' in param else False

    def __str__(self):
        return "%s-%s-%s-%s-%s" % (self.id, self.tuple, str(self.min), str(self.max), self.force_name_matching)

    def is_singleton(self):
        return self.min == self.max

    def is_compatible(self, concert_client):
        '''
          Checks to see if a client is compatible for the implementation's node rule.
        '''
        #print "****** _match ******"
        #print "%s-%s-%s-%s-%s" % (self.id, self.tuple, str(self.min), str(self.max), self.force_name_matching)
        #print concert_client.name + "-" + concert_client.platform + "." + concert_client.system + "." + concert_client.robot
        parts = self.tuple.split('.')
        os = parts[0]
        unused_version = parts[1]
        system = parts[2]
        platform = parts[3]
        app_name = parts[4]
        if os != rocon_std_msgs.PlatformInfo.OS_ANY and os != concert_client.os:
            return False
        # Don't worry about version for now.
        if system != rocon_std_msgs.PlatformInfo.SYSTEM_ANY and system != concert_client.system:
            return False
        if platform != rocon_std_msgs.PlatformInfo.PLATFORM_ANY and platform != concert_client.platform:
            return False
        # Check if we should match name prefixes (i.e. ignore trailing numericals)
        if self.force_name_matching and not re.match(self.id, re.sub('[0-9]*$', '', concert_client.name)):
            return False
        for client_app in concert_client.apps:
            if app_name == client_app.name:
                return True
        return False
