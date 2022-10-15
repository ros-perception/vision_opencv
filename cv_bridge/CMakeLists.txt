cmake_minimum_required(VERSION 3.5)
project(cv_bridge)

find_package(ament_cmake_ros REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(NOT ANDROID)
  find_package(PythonLibs REQUIRED)
  if(PYTHONLIBS_VERSION_STRING VERSION_LESS 3)
    find_package(Boost REQUIRED python)
  else()
    find_package(Boost QUIET)
    message(STATUS "Boost_VERSION_STRING: " ${Boost_VERSION_STRING})
    if(Boost_VERSION_STRING VERSION_LESS "1.67")
      # This is a bit of a hack to suppress a warning
      #   No header defined for python3; skipping header check
      # Which should only affect Boost versions < 1.67
      # Resolution for newer versions:
      #  https://gitlab.kitware.com/cmake/cmake/issues/16391
      set(_Boost_PYTHON3_HEADERS "boost/python.hpp")
      find_package(Boost REQUIRED python3)
    else()
      string(REPLACE "." ";" VERSION_LIST ${PYTHONLIBS_VERSION_STRING})
      list(GET VERSION_LIST 0 PYTHONLIBS_VERSION_MAJOR)
      list(GET VERSION_LIST 1 PYTHONLIBS_VERSION_MINOR)
      find_package(Boost COMPONENTS python${PYTHONLIBS_VERSION_MAJOR}${PYTHONLIBS_VERSION_MINOR} REQUIRED)
    endif()
  endif()
else()
  find_package(Boost REQUIRED)
endif()

find_package(rcpputils REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV 4 QUIET
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3 REQUIRED
    COMPONENTS
      opencv_core
      opencv_imgproc
      opencv_imgcodecs
    CONFIG
  )
endif()

if(NOT ANDROID)
  ament_python_install_package(${PROJECT_NAME}
    PACKAGE_DIR python/${PROJECT_NAME}
  )
endif()

add_subdirectory(src)

# cv_bridge_lib_dir is passed as APPEND_LIBRARY_DIRS for each ament_add_gtest call so
# the project library that they link against is on the library path.
# This is especially important on Windows.
# This is overwritten each loop, but which one it points to doesn't really matter.
set(cv_bridge_lib_dir "$<TARGET_FILE_DIR:${PROJECT_NAME}>")

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_export_dependencies(
  OpenCV
  sensor_msgs
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# install the include folder
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

ament_package(
  CONFIG_EXTRAS "cmake/cv_bridge-extras.cmake.in"
)
