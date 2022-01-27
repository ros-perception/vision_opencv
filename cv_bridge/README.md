cv_bridge
==========

# Introduction

cv_bridge converts between ROS 2 image messages and OpenCV image representation for perception applications. As follows:

![cv_bridge overview](http://wiki.ros.org/cv_bridge?action=AttachFile&do=get&target=cvbridge.png)

This ros2 branch initially derives from porting on the basis of [ros kinetic branch](https://github.com/ros-perception/vision_opencv/tree/kinetic/cv_bridge)

# Installation

Firstly, it assumes that the `ROS 2 core` has already been installed, please refer to [ROS 2 installation](https://docs.ros.org/en/rolling/Installation.html) to get started.

## Install dependencies
OpenCV3+ is a must to install, please refer to the official installation guide from [OpenCV Tutorials](http://docs.opencv.org/master/d9/df8/tutorial_root.html)
Since ROS 2 uses Python 3, please make sure that python3-numpy is installed, or install like this:

```bash

sudo apt install python3-numpy

```

The cv_bridge python backend still has a dependency on python boost (`equal or higher than 1.58.0`), and install them as follows in Ubuntu:

```bash

sudo apt install libboost-python-dev

```

## Build and Test

### Fetch the latest code and build
```bash

cd <YOUR_ROS2_WORKSPACE>
git clone https://github.com/ros-perception/vision_opencv.git
cd vision_opencv
git checkout ros2
colcon build --symlink-install

```

### Run the tests
Python tests have a dependency on OpenCV Python support.  To install it:
```bash
pip3 install opencv-python

```
Next to prepare runtime environment and run tests:
```bash

source <YOUR_ROS2_WORKSPACE>/install/local_setup.bash
colcon test

```

# Known issues
* `boost endian`: remove boost endian APIs with standard C++ 11 or higher instead
* Not tested with Windows or macOS environments so there may be issues building or running
