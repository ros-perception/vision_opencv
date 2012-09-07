project(cv_bridge)
catkin_project(${PROJECT_NAME} INCLUDE_DIRS include
                               LIBRARIES ${PROJECT_NAME}
)


find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED roscpp sensor_msgs)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS}
                           ${roscpp_INCLUDE_DIRS}
                           ${sensor_msgs_INCLUDE_DIRS}
)
include_directories(include)

# add library
add_library(${PROJECT_NAME} SHARED src/cv_bridge.cpp)
target_link_libraries(${PROJECT_NAME}  ${OpenCV_LIBS})

install(DIRECTORY include
        DESTINATION ${CATKIN_PROJECT_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PROJECT_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
)

# add a test
catkin_add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME}
                                            ${OpenCV_LIBRARIES}
)
