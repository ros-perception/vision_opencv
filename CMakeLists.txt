cmake_minimum_required(VERSION 2.8)

# find_package(catkin REQUIRED)
# catkin_stack(ros_comm)

# optional find_package(... [REQUIRED])

# optional catkin_python_setup()

foreach(subdir
    cv_bridge
    )
  add_subdirectory(${subdir})
endforeach()
