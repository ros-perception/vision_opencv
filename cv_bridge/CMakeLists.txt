project(cv_bridge)
catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)


find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED sensor_msgs)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS}
                           ${sensor_msgs_INCLUDE_DIRS}
)
include_directories(include)

# add library
add_library(${PROJECT_NAME} SHARED src/cv_bridge.cpp)
target_link_libraries(${PROJECT_NAME}  ${OpenCV_LIBS})

install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# add the tests
catkin_add_gtest(${PROJECT_NAME}-utest test/utest.cpp test/utest2.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME}
                                            ${OpenCV_LIBRARIES}
)
