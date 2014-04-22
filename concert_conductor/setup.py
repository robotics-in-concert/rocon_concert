#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['concert_conductor'],
    package_dir={'': 'src'},
    scripts=['scripts/concert_conductor_graph',
             ],
    requires=['std_msgs', 'rospy', 'rosgraph', 'rocon_app_manager_msgs', 'concert_msgs', 'gateway_msgs', 'rocon_utilities']
)

setup(**d)
