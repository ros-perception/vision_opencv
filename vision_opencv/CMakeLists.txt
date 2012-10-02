cmake_minimum_required(VERSION 2.8)
project(image_pipeline)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED camera_calibration cv_bridge)
catkin_package(DEPENDS camera_calibration cv_bridge)
