configure_file(__init__.py.plain.in
  ${CMAKE_CURRENT_BINARY_DIR}/boost/__init__.py
  @ONLY
)

ament_python_install_package(${PROJECT_NAME})
