#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['concert_schedulers'],
    package_dir={'': 'src'},
    scripts=['scripts/rocon_scheduler_requests'
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
