cmake_minimum_required(VERSION 2.8.3)
project(catkinized_libav)
 
set(CMAKE_BUILD_TYPE Release)
 
find_package(catkin REQUIRED)
 
include(ExternalProject)
 
set(VERSION 2.4.1)
 
catkin_package(
#DEPENDS
#CATKIN_DEPENDS
#INCLUDE_DIRS include
LIBRARIES libav_wrap
)

ExternalProject_Add(libav_trunk
  PREFIX libav_trunk
  GIT_REPOSITORY https://github.com/libav/libav.git
  GIT_TAG v9.8
  DOWNLOAD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libav_trunk"
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libav_trunk"
  CONFIGURE_COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/libav_trunk/configure" --enable-shared --enable-libx264 --enable-gpl --prefix="${CATKIN_DEVEL_PREFIX}"
)

add_library(libav_wrap src/wrap_lib.cc)
SET(LIBAV_LIBRARY_DEST ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION})
target_link_libraries(libav_wrap ${LIBAV_LIBRARY_DEST}/libavcodec.so ${LIBAV_LIBRARY_DEST}/libavdevice.so ${LIBAV_LIBRARY_DEST}/libavfilter.so ${LIBAV_LIBRARY_DEST}/libavformat.so ${LIBAV_LIBRARY_DEST}/libavresample.so ${LIBAV_LIBRARY_DEST}/libavutil.so ${LIBAV_LIBRARY_DEST}/libswscale.so)
add_dependencies(libav_wrap libav_trunk)
 
install(DIRECTORY include
DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
install(DIRECTORY lib
DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
)
install(DIRECTORY share
DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}
)
