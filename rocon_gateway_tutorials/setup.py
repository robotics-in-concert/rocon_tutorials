#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rocon_gateway_tutorials'],
    package_dir={'': 'src'},
    #scripts=['scripts/gateway_info',
    #         'scripts/remote_gateway_info'
    #         ],
    requires=['rocon_hub', 'rocon_gateway', 'rocon_gateway_graph', 'zeroconf_avahi']
)

setup(**d)
