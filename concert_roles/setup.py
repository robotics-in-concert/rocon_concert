#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['concert_roles'],
    package_dir={'': 'src'},
    scripts=['scripts/concert_roles_and_apps',
             ],
    requires=['std_msgs', 
              'rospy', 
              'concert_msgs', 
              'rocon_utilities',
              'rocon_std_msgs'
             ]
)

setup(**d)
