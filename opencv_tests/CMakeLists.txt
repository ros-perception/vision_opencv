cmake_minimum_required(VERSION 2.8)
project(opencv_tests)

find_package(catkin REQUIRED)
catkin_package()

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED cv_bridge)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
