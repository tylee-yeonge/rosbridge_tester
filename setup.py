#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'rosbridge_tester',
        'rosbridge_tester.utility',        
    ],
    package_dir={'': 'src'}
)


setup(**d)
