cmake_minimum_required(VERSION 2.8)
project(vision_opencv)

find_package(catkin REQUIRED)
catkin_stack()

catkin_python_setup()

foreach(subdir
    cv_bridge
    )
  add_subdirectory(${subdir})
endforeach()
