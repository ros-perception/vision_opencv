project(opencv_tests)
catkin_project(${PROJECT_NAME})

find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

catkin_add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${OpenCV_LIBRARIES})

rosbuild_add_pyunit(test/enumerants.py)
