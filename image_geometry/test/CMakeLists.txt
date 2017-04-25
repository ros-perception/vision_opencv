find_package(ament_cmake_nose REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

ament_add_nose_test(directed.py .)

ament_add_gtest(${PROJECT_NAME}-utest utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME} ${OpenCV_LIBS})
