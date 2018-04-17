vision_opencv
=============
# Introduction
ros2 vison_opencv contains packages to interface ROS2 with [OpenCV](http://opencv.org/) which is a library designed for computational efficiency and strong focus for real time computer vision applications. What package it contains:
* `cv_bridge`: Bridge between ROS2 image messages and OpenCV image representation
* `image_geometry`: Collection of methods for dealing with image and pixel geometry
* `opencv_tests`: Integration tests to use the capability of the packages with opencv

In order to use ROS2 with OpenCV, please see the details within [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge) package and OpenCV is a system dependency

# Known issues
Now ros2 cv_bridge is maintained here but the ros2 image_geometry is on the way, OSRF has an usable [image_geometry](https://github.com/ros2/vision_opencv/tree/ros2/image_geometry) and please refer to it to leverage in case of quick practice requirement  

