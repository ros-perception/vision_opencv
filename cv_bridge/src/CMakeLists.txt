# add library
add_library(${PROJECT_NAME}
  cv_bridge.cpp
  cv_mat_sensor_msgs_image_type_adapter.cpp
  rgb_colors.cpp
)
include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME} EXPORT_FILE_NAME ${PROJECT_NAME}/${PROJECT_NAME}_export.h)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${sensor_msgs_TARGETS}
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
  rclcpp::rclcpp
  ament_cmake_ros_core::ament_ros_defaults
)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}_export.h
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})

if(NOT CV_BRIDGE_DISABLE_PYTHON)
  # NB_STATIC builds the nanobind runtime into the module, so nothing else has to be installed.
  nanobind_add_module(_${PROJECT_NAME} NB_STATIC module.cpp)
  target_link_libraries(_${PROJECT_NAME} PRIVATE
    ${PROJECT_NAME}
    ament_cmake_ros_core::ament_ros_defaults)

  install(TARGETS _${PROJECT_NAME} DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME})
endif()
