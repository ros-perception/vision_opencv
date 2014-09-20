cmake_minimum_required(VERSION 2.8)
project(cv_bridge)

find_package(catkin REQUIRED COMPONENTS rosconsole sensor_msgs)

find_package(Boost REQUIRED python)
find_package(OpenCV REQUIRED)
if (OpenCV_VERSION VERSION_EQUAL "3")
  add_definitions("-DOPENCV3=1")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rosconsole sensor_msgs
  DEPENDS OpenCV
)

catkin_python_setup()

include_directories(SYSTEM ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(python)
add_subdirectory(src)
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# install the include folder
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
