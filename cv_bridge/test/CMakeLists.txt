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
ament_add_gtest(test_compression test_compression.cpp)
target_link_libraries(test_compression ${PROJECT_NAME})

ament_add_gtest(test_endian test_endian.cpp)
target_link_libraries(test_endian ${PROJECT_NAME})

ament_add_gtest(test_rgb_colors test_rgb_colors.cpp)
target_link_libraries(test_rgb_colors ${PROJECT_NAME})

ament_add_gtest(utest utest.cpp)
target_link_libraries(utest ${PROJECT_NAME})

ament_add_gtest(utest2 utest2.cpp)
target_link_libraries(utest2 ${PROJECT_NAME})

# enable cv_bridge python tests
find_package(ament_cmake_pytest REQUIRED)
ament_add_pytest_test(enumerants.py "enumerants.py")
ament_add_pytest_test(conversions.py "conversions.py" TIMEOUT 600 ${SKIP_TEST})
ament_add_pytest_test(python_bindings.py "python_bindings.py")
