cv_bridge
==========

# Introduction 

cv_bridge converts between ROS2 image messages and OpenCV image representation for perception applications. As follows:

![cv_bridge overview](http://wiki.ros.org/cv_bridge?action=AttachFile&do=get&target=cvbridge.png)

This ros2 branch initially derives from porting on the basis of [ros kinetic branch](https://github.com/ros-perception/vision_opencv/tree/kinetic/cv_bridge)

# Installation

Firstly, it assumes that the `ros2 core` has already been installed, please refer to [ROS2 installation](https://github.com/ros2/ros2/wiki/Installation) to get started if the `ros2 core` isn't ready to use

## Install dependencies
OpenCV3 is a must to install, please refer to the official installation guide from [OpenCV Tutorials](http://docs.opencv.org/master/d9/df8/tutorial_root.html)
Since ros2 bases on python3, please make sure that python3-numpy is installed, or install like this:

```bash

sudo apt install python3-numpy

```

Now cv_bridge python backend still has dependency on python boost (`equal or higer than 1.58.0`), and install them as follows in Ubuntu 16.04:

```bash

sudo apt install libboost-python1.58.0
cd /usr/lib/x86_64-linux-gnu/ && sudo ln -s libboost_python-py35.so libboost_python3.so

```

## Build and Test

### Fetch the latest code and build 
```bash

cd <YOUR_ROS2_WORKSPACE>
git clone https://github.com/ros-perception/vision_opencv.git
git checkout ros2
src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install

```

### Run the tests
Python tests have dependency to opencv python support and install it:
```bash
pip3 install opencv-python

```
Next to prepare runtime environment and run tests:
```bash

source <YOUR_ROS2_WORKSPACE>/install/local_setup.bash
ament test --skip-build --skip-install --only-packages cv_bridge

```

# Known issues
* `boost endian`: remove boost endian APIs with standard C++ 11 or higer instead 
* Not verify with Windows and OS X environment and there may be building or running issues
