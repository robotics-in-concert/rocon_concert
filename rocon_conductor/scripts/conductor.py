#!/usr/bin/env python

import roslib; roslib.load_manifest('rocon_conductor')
import rospy

from ros import rocon_conductor


if __name__ == '__main__':
    rocon_conductor.conductor.main()
