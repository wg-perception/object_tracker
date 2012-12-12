if (ROS_GROOVY_FOUND)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/__init__.py.plain.in
               ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py
               @ONLY
)

install(FILES ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py
)
else()
set(ECTO_PYTHON_BUILD_PATH ${CMAKE_BINARY_DIR}/gen/py/)

configure_file(__init__.py.plain.in
               ${ECTO_PYTHON_BUILD_PATH}/${PROJECT_NAME}/__init__.py
               @ONLY
)

install(FILES ${ECTO_PYTHON_BUILD_PATH}/${PROJECT_NAME}/__init__.py
        DESTINATION ${ECTO_PYTHON_INSTALL_PATH}/${PROJECT_NAME}/
)
endif()
