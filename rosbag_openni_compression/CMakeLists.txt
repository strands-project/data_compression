cmake_minimum_required(VERSION 2.8.3)
project(rosbag_openni_compression)

find_package(catkin REQUIRED
  actionlib
  actionlib_msgs
  rospy)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  RosbagRecordCamera.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
)


catkin_package(
# INCLUDE_DIRS include
# LIBRARIES strands_utils
# CATKIN_DEPENDS other_catkin_pkg
# DEPENDS system_lib
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/rosbag_record_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
