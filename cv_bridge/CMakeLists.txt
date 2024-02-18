cmake_minimum_required(VERSION 3.14)
project(cv_bridge)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

option(CV_BRIDGE_DISABLE_PYTHON "Disable building Python bindings" OFF)

if(ANDROID)
  set(CV_BRIDGE_DISABLE_PYTHON ON)
endif()

if(CV_BRIDGE_DISABLE_PYTHON)
  find_package(Boost REQUIRED)
  set(boost_python_target "")
else()
  find_package(Python3 REQUIRED COMPONENTS Development NumPy)
  find_package(Boost QUIET)
  if(Boost_VERSION_STRING VERSION_LESS "1.67")
    # This is a bit of a hack to suppress a warning
    #   No header defined for python3; skipping header check
    # Which should only affect Boost versions < 1.67
    # Resolution for newer versions:
    #  https://gitlab.kitware.com/cmake/cmake/issues/16391
    set(_Boost_PYTHON3_HEADERS "boost/python.hpp")
    find_package(Boost REQUIRED COMPONENTS python3)
    set(boost_python_target "Boost::python3")
  else()
    find_package(Boost REQUIRED COMPONENTS python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR})
    set(boost_python_target "Boost::python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR}")
  endif()
endif()

if(NOT CV_BRIDGE_DISABLE_PYTHON)
  ament_python_install_package(${PROJECT_NAME}
    PACKAGE_DIR python/${PROJECT_NAME}
  )
endif()

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/cv_bridge.cpp
  src/cv_mat_sensor_msgs_image_type_adapter.cpp
  src/rgb_colors.cpp
)
include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME} EXPORT_FILE_NAME ${PROJECT_NAME}/${PROJECT_NAME}_export.h)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}_export.h
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})

if(NOT CV_BRIDGE_DISABLE_PYTHON)
  python3_add_library(${PROJECT_NAME}_boost MODULE src/module.cpp src/module_opencv4.cpp)
  target_link_libraries(${PROJECT_NAME}_boost PRIVATE
    ${PROJECT_NAME}
    ${boost_python_target}
    Python3::NumPy)
  target_compile_definitions(${PROJECT_NAME}_boost PRIVATE PYTHON3)

  if(OpenCV_VERSION_MAJOR VERSION_EQUAL 4)
    target_compile_definitions(${PROJECT_NAME}_boost PRIVATE OPENCV_VERSION_4)
  endif()

  set_target_properties(${PROJECT_NAME}_boost PROPERTIES
                        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/boost/
                        PREFIX ""
  )
  install(TARGETS ${PROJECT_NAME}_boost DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/boost/)
endif()

# cv_bridge_lib_dir is passed as APPEND_LIBRARY_DIRS for each ament_add_gtest call so
# the project library that they link against is on the library path.
# This is especially important on Windows.
# This is overwritten each loop, but which one it points to doesn't really matter.
set(cv_bridge_lib_dir "$<TARGET_FILE_DIR:${PROJECT_NAME}>")

if(BUILD_TESTING)
  # Add all the unit tests for cv_bridge
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package()
