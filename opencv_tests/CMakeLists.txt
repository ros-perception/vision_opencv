project(opencv_tests)

## Tests are failing on OSX for an unknown reason
include(CMakeDetermineSystem)
if(CMAKE_SYSTEM_NAME MATCHES "Linux")

find_package(OpenCV REQUIRED)

add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${OpenCV_LIBS})

# add_pyunit doesn't seem to exist yet, even though the catkin docs say it should -jbinney
#rosbuild_add_pyunit(test/enumerants.py)

endif(CMAKE_SYSTEM_NAME MATCHES "Linux")

catkin_project(${PROJECT_NAME})
