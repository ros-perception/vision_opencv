from setuptools import find_packages
from setuptools import setup

package_name = 'opencv_tests'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['launch','setuptools'],
    author='Ethan Gao',
    author_email='ethan.gao@linux.intel.com',
    maintainer='Ethan Gao',
    maintainer_email='ethan.gao@linux.intel.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'opencv tests using cv_bridge and ros2 node implementation'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'source = nodes.source:main',
            'broadcast = nodes.broadcast:main',
            'rosfacedetect = nodes.rosfacedetect:main',
        ],
    },
)
