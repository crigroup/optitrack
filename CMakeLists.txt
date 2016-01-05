cmake_minimum_required(VERSION 2.8.3)
project(optitrack)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  message_generation
  rviz
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  RigidBody.msg
  RigidBodyArray.msg
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  interactive_markers
  message_runtime
  rviz
  sensor_msgs 
  std_msgs
  visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# The plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
# Avoid defining "emit", "slots" for the Qt signals and slots because
# they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)
# Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  src/rigid_body_display.h
)
# Here we specify the list of source files, including the output of
# the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/rigid_body_display.cpp
  ${MOC_FILES}
)
# An rviz plugin is just a shared library
add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} optitrack_gencpp)

# Install rules
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
