cmake_minimum_required(VERSION 3.5)
project(image_geometry)

find_package(ament_cmake_ros REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(OpenCV REQUIRED COMPONENTS calib3d core highgui imgproc)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# add a library
add_library(${PROJECT_NAME}
  src/pinhole_camera_model.cpp src/stereo_camera_model.cpp
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  "OpenCV"
  "sensor_msgs"
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "IMAGE_GEOMETRY_BUILDING_DLL")

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

# image_geometry_lib_dir is passed as APPEND_LIBRARY_DIRS for each ament_add_gtest call so
# the project library that they link against is on the library path.
# This is especially important on Windows.
# This is overwritten each loop, but which one it points to doesn't really matter.
set(image_geometry_lib_dir "$<TARGET_FILE_DIR:${PROJECT_NAME}>")

# add tests
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(OpenCV)
ament_export_include_directories(include)
ament_package()
