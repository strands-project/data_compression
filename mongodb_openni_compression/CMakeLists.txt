cmake_minimum_required(VERSION 2.8.3)
project(mongodb_openni_compression)

find_package(catkin REQUIRED)

catkin_package(
# INCLUDE_DIRS include
# LIBRARIES strands_utils
# CATKIN_DEPENDS other_catkin_pkg
# DEPENDS system_lib
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
