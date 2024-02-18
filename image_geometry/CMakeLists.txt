cmake_minimum_required(VERSION 3.5)
project(image_geometry)

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

include_directories(include)

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/pinhole_camera_model.cpp
  src/stereo_camera_model.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
  target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME})
endif()

ament_auto_package()
