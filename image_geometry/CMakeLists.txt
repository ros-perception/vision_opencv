cmake_minimum_required(VERSION 3.5)
project(image_geometry)

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(OpenCV REQUIRED COMPONENTS calib3d core highgui imgproc)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/pinhole_camera_model.cpp
  src/stereo_camera_model.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  opencv_calib3d
  opencv_core
  opencv_highgui
  opencv_imgproc
  ${sensor_msgs_TARGETS})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "IMAGE_GEOMETRY_BUILDING_DLL")

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
  target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(OpenCV)
ament_export_dependencies(sensor_msgs)

ament_package()
