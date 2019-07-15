#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_setup = generate_distutils_setup(
    packages=['elevator_navigation'],
    package_dir={'elevator_navigation': 'ros/src/elevator_navigation'}
)

setup(**package_setup)
