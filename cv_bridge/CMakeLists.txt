project(cv_bridge)

find_package(ROS REQUIRED roscpp sensor_msgs)
include_directories(${ROS_INCLUDE_DIRS})

include_directories(include)

find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME} SHARED src/cv_bridge.cpp)
target_link_libraries(${PROJECT_NAME}  ${OpenCV_LIBS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  )

add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  )
