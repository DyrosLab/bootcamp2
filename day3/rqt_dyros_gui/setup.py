#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages = ['rqt_dyros_gui'],
    package_dir = {'': 'src'},
    requires = ['std_msgs', 'roscpp'],
    scripts = ['scripts/rqt_dyros_gui']
)

setup(**d)