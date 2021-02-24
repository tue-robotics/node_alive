#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    #  don't do this unless you want a globally visible script
    scripts=['scripts/node_alive_add', 'scripts/node_alive_server'],
    packages=['node_alive'],
    package_dir={'': 'src'}
)

setup(**d)
