#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
  packages=['pick_up_object'],
  package_dir={'': 'src'}
)

setup(**setup_args)