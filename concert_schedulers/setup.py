#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['concert_schedulers', 'concert_schedulers.common', 'concert_schedulers.compatibility_tree_scheduler', 'concert_schedulers.resource_pool_requester'],
    package_dir={'': 'src'},
    scripts=['scripts/concert_scheduler_requests'
             ],
    requires=['std_msgs',
              'rospy',
              'rosgraph',
              'concert_msgs',
              'gateway_msgs',
              'scheduler_msgs',
              'rocon_utilities',
              'unique_id',
              ]
)

setup(**d)
