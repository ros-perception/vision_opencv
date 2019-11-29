cmake_minimum_required(VERSION 3.5)
project(cv_bridge)

find_package(ament_cmake_ros REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(NOT ANDROID)
  find_package(PythonLibs)
  if(PYTHONLIBS_VERSION_STRING VERSION_LESS 3)
    find_package(Boost REQUIRED python)
  else()
    find_package(Boost QUIET)
    if(Boost_VERSION LESS 106500)
      find_package(Boost REQUIRED python)
    else()
      # This is a bit of a hack to suppress a warning
      #   No header defined for python3; skipping header check
      # Which should only affect Boost versions < 1.67
      # Resolution for newer versions:
      #  https://gitlab.kitware.com/cmake/cmake/issues/16391
      if (Boost_VERSION LESS 106700)
        set(_Boost_PYTHON3_HEADERS "boost/python.hpp")
      endif()
      find_package(Boost COMPONENTS python3 REQUIRED)
    endif()
  endif()
else()
  find_package(Boost REQUIRED)
endif()

find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)

include_directories(include ${Boost_INCLUDE_DIRS})

if(NOT ANDROID)
  add_subdirectory(python)
endif()

add_subdirectory(src)

# cv_bridge_lib_dir is passed as APPEND_LIBRARY_DIRS for each ament_add_gtest call so
# the project library that they link against is on the library path.
# This is especially important on Windows.
# This is overwritten each loop, but which one it points to doesn't really matter.
set(cv_bridge_lib_dir "$<TARGET_FILE_DIR:${PROJECT_NAME}>")

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_export_dependencies(OpenCV)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# install the include folder
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package(
  CONFIG_EXTRAS "cmake/cv_bridge-extras.cmake.in"
)
