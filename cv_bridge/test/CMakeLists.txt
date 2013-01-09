# add the tests
catkin_add_gtest(${PROJECT_NAME}-utest utest.cpp utest2.cpp)
target_link_libraries(${PROJECT_NAME}-utest
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

catkin_add_nosetests(enumerants.py)
