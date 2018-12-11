#!/usr/bin/env python
from setuptools import find_packages
from setuptools import setup

package_name = 'image_geometry'

setup(
    name=package_name,
    version='2.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    author='Tony Guo',
    author_email='tony.guo@intel.com',
    maintainer='Tony Guo',
    maintainer_email='tony.guo@intel.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'object map implementation for ROS2'
    ),
    license='Apache License, Version 2.0',
)