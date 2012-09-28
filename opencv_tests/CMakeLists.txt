project(opencv_tests)
catkin_package()

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED cv_bridge)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

catkin_add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${cv_bridge_LIBRARIES}
                                            ${OpenCV_LIBRARIES}
)

catkin_add_nosetests(test/enumerants.py)
