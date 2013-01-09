configure_file(__init__.py.plain.in
               ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/boost/__init__.py
               @ONLY
)

install(FILES ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/boost/__init__.py
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/boost/
)
