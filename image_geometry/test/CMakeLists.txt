find_package(ament_cmake_gtest REQUIRED)
# TODO(mikaelarguedas) Reenable this when tests are ported to ROS 2
# find_package(ament_cmake_pytest REQUIRED)

# ament_add_pytest_test(directed.py "directed.py")

ament_add_gtest(${PROJECT_NAME}-utest utest.cpp APPEND_LIBRARY_DIRS "${image_geometry_lib_dir}")
target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME} ${OpenCV_LIBS})
