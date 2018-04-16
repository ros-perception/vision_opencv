from setuptools import find_packages
from setuptools import setup

package_name = 'cv_bridge'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
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
        'opencv bridge python implementation '
        'but are now just used for demo purposes.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'conversions = test.conversions:main',
            'enumerants = test.enumerants:main',
        ],
    },
)
