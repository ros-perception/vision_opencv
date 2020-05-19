# add library
include_directories(./)
add_library(${PROJECT_NAME} cv_bridge.cpp rgb_colors.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

if(NOT ANDROID)
# add a Boost Python library
find_package(PythonInterp REQUIRED)
find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")

#Get the numpy include directory from its python module
if(NOT PYTHON_NUMPY_INCLUDE_DIR)
    execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
                    RESULT_VARIABLE PYTHON_NUMPY_PROCESS
                    OUTPUT_VARIABLE PYTHON_NUMPY_INCLUDE_DIR
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(PYTHON_NUMPY_PROCESS EQUAL 0)
       file(TO_CMAKE_PATH "${PYTHON_NUMPY_INCLUDE_DIR}" PYTHON_NUMPY_INCLUDE_CMAKE_PATH)
       set(PYTHON_NUMPY_INCLUDE_DIR ${PYTHON_NUMPY_INCLUDE_CMAKE_PATH} CACHE PATH "Numpy include directory")
    else(PYTHON_NUMPY_PROCESS EQUAL 0)
        message(SEND_ERROR "Could not determine the NumPy include directory, verify that NumPy was installed correctly.")
    endif(PYTHON_NUMPY_PROCESS EQUAL 0)
 endif(NOT PYTHON_NUMPY_INCLUDE_DIR)

include_directories(${PYTHON_INCLUDE_PATH} ${Boost_INCLUDE_DIRS} ${PYTHON_NUMPY_INCLUDE_DIR})

if (PYTHON_VERSION_MAJOR VERSION_EQUAL 3)
  add_definitions(-DPYTHON3)
endif()

add_library(${PROJECT_NAME}_boost module.cpp module_opencv4.cpp)
target_link_libraries(${PROJECT_NAME}_boost ${Boost_LIBRARIES}
                                            ${catkin_LIBRARIES}
                                            ${PROJECT_NAME}
)

if(NOT APPLE)
  target_link_libraries(${PROJECT_NAME}_boost ${PYTHON_LIBRARIES})
endif()

set_target_properties(${PROJECT_NAME}_boost PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME}/boost/
                      RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME}/boost/
                      PREFIX ""
)
if(APPLE)
  set_target_properties(${PROJECT_NAME}_boost PROPERTIES
                        SUFFIX ".so")
  set_target_properties(${PROJECT_NAME}_boost PROPERTIES
                        LINK_FLAGS "-undefined dynamic_lookup")
endif()
if(MSVC)
  set_target_properties(${PROJECT_NAME}_boost PROPERTIES
                        SUFFIX ".pyd")
endif()

install(TARGETS ${PROJECT_NAME}_boost DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/boost/)
endif()
