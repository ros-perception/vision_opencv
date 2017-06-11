vision_opencv
=============

.. image:: https://travis-ci.org/ros-perception/vision_opencv.svg?branch=indigo
    :target: https://travis-ci.org/ros-perception/vision_opencv

Fork of github.com/ros-perception/vision_opencv

This package has no dependency on opencv through rosdep.  It still requires OpenCV to build, and CMake will complain if it can't find OpenCV.  However, it avoids the messiness of ROS's OpenCV packages.

The kinetic branch here tracks the upstream kinetic branch, the branch to use is no-opencv-rosdep.

Packages for interfacing ROS with OpenCV, a library of programming functions for real time computer vision.
