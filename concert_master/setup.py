#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['concert_master'],
    package_dir={'': 'src'},
    scripts=['scripts/concert_info',
             ],
    requires=['rospy', 'concert_msgs', 'rocon_utilities']
)

setup(**d)
