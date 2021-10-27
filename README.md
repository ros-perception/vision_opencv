vision_opencv
=============
ros2 vision_opencv contains packages to interface ROS 2 with [OpenCV](http://opencv.org/) which is a library designed for computational efficiency and strong focus for real time computer vision applications. This repository contains:
* `cv_bridge`: Bridge between ROS 2 image messages and OpenCV image representation
* `image_geometry`: Collection of methods for dealing with image and pixel geometry
* `opencv_tests`: Integration tests to use the capability of the packages with opencv
* `vision_opencv`: Meta-package to install both `cv_bridge` and `image_geometry`

In order to use ROS 2 with OpenCV, please see the details within [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge) package.
