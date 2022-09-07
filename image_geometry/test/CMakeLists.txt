find_package(ament_cmake_gtest REQUIRED)
# TODO(mikaelarguedas) Reenable this when tests are ported to ROS 2
find_package(ament_cmake_pytest REQUIRED)
find_package(ament_cmake_flake8 REQUIRED)
find_package(ament_cmake_pep257 REQUIRED)
find_package(ament_cmake_copyright REQUIRED)

ament_add_pytest_test(directed.py "directed.py")
ament_copyright()
ament_pep257()
ament_flake8()

ament_add_gtest(${PROJECT_NAME}-utest utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}-utest-equi utest_equi.cpp)
target_link_libraries(${PROJECT_NAME}-utest-equi ${PROJECT_NAME})
