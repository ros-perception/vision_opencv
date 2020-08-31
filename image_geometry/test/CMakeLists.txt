catkin_add_nosetests(directed.py)

catkin_add_gtest(${PROJECT_NAME}-utest utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME} ${OpenCV_LIBS})

catkin_add_gtest(${PROJECT_NAME}-utest-equi utest_equi.cpp)
target_link_libraries(${PROJECT_NAME}-utest-equi ${PROJECT_NAME} ${OpenCV_LIBS})
