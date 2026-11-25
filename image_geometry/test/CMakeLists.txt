find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_add_pytest_test(directed.py "directed.py")

ament_add_gtest(${PROJECT_NAME}-utest utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}-utest-equi utest_equi.cpp)
target_link_libraries(${PROJECT_NAME}-utest-equi ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}-utest-stereo utest_stereo.cpp)
target_link_libraries(${PROJECT_NAME}-utest-stereo ${PROJECT_NAME})
