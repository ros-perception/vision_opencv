cmake_minimum_required(VERSION 3.18)
project(cv_bridge)

find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

option(CV_BRIDGE_DISABLE_PYTHON "Disable building Python bindings" OFF)

if(ANDROID)
  set(CV_BRIDGE_DISABLE_PYTHON ON)
endif()

if(NOT CV_BRIDGE_DISABLE_PYTHON)
  # nanobind needs the targets of FindPython. Use the interpreter the rest of the build uses.
  if(Python3_EXECUTABLE AND NOT Python_EXECUTABLE)
    set(Python_EXECUTABLE "${Python3_EXECUTABLE}")
  endif()
  find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
  find_package(nanobind CONFIG REQUIRED)
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)

if(NOT CV_BRIDGE_DISABLE_PYTHON)
  ament_python_install_package(${PROJECT_NAME}
    PACKAGE_DIR python/${PROJECT_NAME}
  )
endif()

add_subdirectory(src)

# cv_bridge_lib_dir is passed as APPEND_LIBRARY_DIRS for each ament_add_gtest call so
# the project library that they link against is on the library path.
# This is especially important on Windows.
# This is overwritten each loop, but which one it points to doesn't really matter.
set(cv_bridge_lib_dir "$<TARGET_FILE_DIR:${PROJECT_NAME}>")

if(BUILD_TESTING)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # Setting this flag disables the copyright check
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ament_cmake_ros_core
  OpenCV
  sensor_msgs
  rclcpp
)

ament_export_targets(export_${PROJECT_NAME})

# install the include folder
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

ament_package(
  CONFIG_EXTRAS "cmake/cv_bridge-extras.cmake.in"
)
