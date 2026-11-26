cmake_minimum_required(VERSION 3.14)
project(image_geometry)

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(OpenCV REQUIRED COMPONENTS core)
if(OpenCV_VERSION_MAJOR VERSION_GREATER_EQUAL 5)
  set(OPENCV_CALIB3D_COMPONENT calib)
else()
  set(OPENCV_CALIB3D_COMPONENT calib3d)
endif()
find_package(OpenCV REQUIRED COMPONENTS ${OPENCV_CALIB3D_COMPONENT} core imgproc)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/pinhole_camera_model.cpp
  src/stereo_camera_model.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  opencv_${OPENCV_CALIB3D_COMPONENT}
  opencv_core
  opencv_imgproc
  ${sensor_msgs_TARGETS}
  ament_cmake_ros_core::ament_ros_defaults)

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
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  # The copyright check is disabled. cpplint is run below, without its own copyright check.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-legal/copyright")
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(ament_cmake_ros_core)
ament_export_dependencies(OpenCV)
ament_export_dependencies(sensor_msgs)

ament_package()
