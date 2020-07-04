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
  find_package(PythonLibs)
  if(PYTHONLIBS_VERSION_STRING VERSION_LESS 3)
    find_package(Boost REQUIRED python)
  else()
    find_package(Boost QUIET)
    if(Boost_VERSION LESS 106500)
      find_package(Boost REQUIRED python)
    else()
      # This is a bit of a hack to suppress a warning
      #   No header defined for python3; skipping header check
      # Which should only affect Boost versions < 1.67
      # Resolution for newer versions:
      #  https://gitlab.kitware.com/cmake/cmake/issues/16391
      if (Boost_VERSION LESS 106700)
        set(_Boost_PYTHON3_HEADERS "boost/python.hpp")
      endif()
      # Boost >= 1.67 requires python suffix, like "python27" or "python38"
      # (see https://cmake.org/cmake/help/latest/module/FindBoost.html?highlight=version%20suffix)
      # In the future, for CMAKE > 3.12, FindPython3 and Python3_VERSION_MAJOR etc. could be to build suffix
      string(REPLACE "." ";" PYTHONLIBS_VERSION_LIST ${PYTHONLIBS_VERSION_STRING})
      list(GET PYTHONLIBS_VERSION_LIST 0 PYTHONLIBS_VERSION_MAJOR)
      list(GET PYTHONLIBS_VERSION_LIST 1 PYTHONLIBS_VERSION_MINOR)
      set(PYTHON_SUFFIX "python${PYTHONLIBS_VERSION_MAJOR}${PYTHONLIBS_VERSION_MINOR}")
      find_package(Boost COMPONENTS ${PYTHON_SUFFIX} REQUIRED)
    endif()
  endif()
else()
  find_package(Boost REQUIRED)
endif()

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
  add_subdirectory(python)
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

ament_export_dependencies(OpenCV)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# install the include folder
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package(
  CONFIG_EXTRAS "cmake/cv_bridge-extras.cmake.in"
)
