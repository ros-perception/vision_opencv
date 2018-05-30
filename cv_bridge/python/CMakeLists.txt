configure_file(__init__.py.plain.in
  ${CMAKE_CURRENT_BINARY_DIR}/boost/__init__.py
  @ONLY
)

ament_python_install_package(${PROJECT_NAME})

# install(FILES ${CMAKE_CURRENT_BINARY_DIR}/boost/__init__.py
#   DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/boost/
# )
