from setuptools import find_packages
from setuptools import setup

package_name = 'opencv_tests'

setup(
    name=package_name,
    version='3.2.1',
    packages=find_packages(exclude=['launch']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ethan Gao',
    author_email='ethan.gao@linux.intel.com',
    maintainer='Kenji Brameld',
    maintainer_email='kenjibrameld@gmail.com',
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
            'source = opencv_tests.source:main',
            'broadcast = opencv_tests.broadcast:main',
            'rosfacedetect = opencv_tests.rosfacedetect:main',
        ],
    },
)
