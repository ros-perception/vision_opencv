cmake_minimum_required(VERSION 2.8)
project(cv_bridge)

find_package(catkin REQUIRED COMPONENTS rosconsole sensor_msgs)

if(NOT ANDROID)
  find_package(PythonLibs)
  if(PYTHONLIBS_VERSION_STRING MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)")
    set(Python_VERSION_MAJOR "${CMAKE_MATCH_1}")
    set(Python_VERSION_MINOR "${CMAKE_MATCH_2}")
    set(Python_VERSION_PATCH "${CMAKE_MATCH_3}")
    find_package(Boost QUIET)
    if(Boost_VERSION LESS 106700)
      if(Python_VERSION_MAJOR LESS 3)
        find_package(Boost REQUIRED COMPONENTS python)
      else()
        find_package(Boost REQUIRED COMPONENTS python3)
      endif()
    else()
      # The boost_python library has been renamed in Boost 1.67 and the FindBoost.cmake
      # module requires a Python version suffix:
      #
      # References:
      # - https://www.boost.org/docs/libs/1_67_0/libs/python/doc/html/rn.html
      # - https://cmake.org/cmake/help/v3.12/module/FindBoost.html
      #
      find_package(Boost REQUIRED COMPONENTS python${Python_VERSION_MAJOR}${Python_VERSION_MINOR})
    endif()
  else()
    message(FATAL_ERROR "Could not detect the Python version from string '${PYTHONLIBS_VERSION_STRING}' required to find the respective boost_python module.")
  endif()
else()
find_package(Boost REQUIRED)
endif()
find_package(OpenCV 3 REQUIRED
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rosconsole sensor_msgs
  DEPENDS OpenCV
  CFG_EXTRAS cv_bridge-extras.cmake
)

catkin_python_setup()

include_directories(include ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

if(NOT ANDROID)
add_subdirectory(python)
endif()
add_subdirectory(src)
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# install the include folder
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
