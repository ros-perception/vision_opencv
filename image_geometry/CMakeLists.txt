project(image_geometry)
catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED roscpp sensor_msgs)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS}
                           ${roscpp_INCLUDE_DIRS}
                           ${sensor_msgs_INCLUDE_DIRS}
)
include_directories(include)

# add a library
add_library(${PROJECT_NAME} src/pinhole_camera_model.cpp src/stereo_camera_model.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

catkin_add_nosetests(test/directed.py)

catkin_add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME} ${OpenCV_LIBS})

install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
