# Add all the unit tests for cv_bridge
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

# skip the conversions.py test when necessary, because its
# test_encode_decode_cv2_compressed and test_encode_decode_cv2
# spends more than 5 minutes generally
if(${SKIP_PYCONVERSION_TEST})
  set(SKIP_TEST "SKIP_TEST")
else()
  set(SKIP_TEST "")
endif()

# enable cv_bridge C++ tests
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(${PROJECT_NAME}-utest
  test_endian.cpp
  test_compression.cpp
  utest.cpp utest2.cpp
  test_rgb_colors.cpp
  APPEND_LIBRARY_DIRS "${cv_bridge_lib_dir}")
target_link_libraries(${PROJECT_NAME}-utest
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

# enable cv_bridge python tests
find_package(ament_cmake_pytest REQUIRED)
ament_add_pytest_test(enumerants.py "enumerants.py")
ament_add_pytest_test(conversions.py "conversions.py" TIMEOUT 600 ${SKIP_TEST})
ament_add_pytest_test(python_bindings.py "python_bindings.py")
