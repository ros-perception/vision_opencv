cmake_minimum_required(VERSION 2.8)
project(cv_bridge)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED rosconsole sensor_msgs)
find_package(OpenCV)
catkin_package(DEPENDS OpenCV rosconsole sensor_msgs
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)

catkin_python_setup()

find_package(Boost REQUIRED)
find_package(catkin REQUIRED rosconsole sensor_msgs)
find_package(OpenCV REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
                           ${rosconsole_INCLUDE_DIRS}
                           ${sensor_msgs_INCLUDE_DIRS}
)
include_directories(include)

# add library
add_library(${PROJECT_NAME} SHARED src/cv_bridge.cpp)
target_link_libraries(${PROJECT_NAME}  ${OpenCV_LIBS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
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
