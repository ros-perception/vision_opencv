# add library
add_library(${PROJECT_NAME} cv_bridge.cpp rgb_colors.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "OpenCV"
  "sensor_msgs"
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

if(NOT ANDROID)
# add a Boost Python library
find_package(PythonInterp REQUIRED)
find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")

# Get the numpy include directory from its python module
if(NOT PYTHON_NUMPY_INCLUDE_DIR)
  execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
    RESULT_VARIABLE PYTHON_NUMPY_PROCESS
    OUTPUT_VARIABLE PYTHON_NUMPY_INCLUDE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  if(PYTHON_NUMPY_PROCESS EQUAL 0)
    file(TO_CMAKE_PATH "${PYTHON_NUMPY_INCLUDE_DIR}" PYTHON_NUMPY_INCLUDE_CMAKE_PATH)
    set(PYTHON_NUMPY_INCLUDE_DIR ${PYTHON_NUMPY_INCLUDE_CMAKE_PATH} CACHE PATH "Numpy include directory")
  else()
    message(SEND_ERROR "Could not determine the NumPy include directory, verify that NumPy was installed correctly.")
  endif()
endif()

include_directories(${PYTHON_INCLUDE_PATH} ${PYTHON_NUMPY_INCLUDE_DIR})

if(PYTHON_VERSION_MAJOR VERSION_EQUAL 3)
  add_definitions(-DPYTHON3)
endif()

if(OpenCV_VERSION_MAJOR VERSION_EQUAL 3)
  add_library(${PROJECT_NAME}_boost module.cpp module_opencv3.cpp)
else()
  add_library(${PROJECT_NAME}_boost module.cpp module_opencv2.cpp)
endif()
target_link_libraries(${PROJECT_NAME}_boost
  ${PYTHON_LIBRARIES}
  ${PROJECT_NAME}
)

set_target_properties(${PROJECT_NAME}_boost PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/boost/
                      PREFIX ""
)
if(APPLE)
  set_target_properties(${PROJECT_NAME}_boost PROPERTIES
                        SUFFIX ".so")
endif()

install(TARGETS ${PROJECT_NAME}_boost DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/boost/)
endif()
