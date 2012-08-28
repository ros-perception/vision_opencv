project(image_geometry)

find_package(ROS REQUIRED roscpp sensor_msgs)
include_directories(${ROS_INCLUDE_DIRS})

include_directories(include)

find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME} src/pinhole_camera_model.cpp src/stereo_camera_model.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# add_pyunit doesn't seem to exist yet, even though the catkin docs say it should -jbinney
#add_pyunit(test/directed.py)

add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME} ${OpenCV_LIBS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  )
