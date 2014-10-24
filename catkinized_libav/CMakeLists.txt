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
#  PREFIX libav_trunk
#  GIT_REPOSITORY https://github.com/libav/libav.git
#  GIT_TAG v9.8
#  DOWNLOAD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libav_trunk"
  DOWNLOAD_COMMAND ""
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libav_trunk"
  CONFIGURE_COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/libav_trunk/configure" --enable-shared --enable-libx264 --enable-gpl --prefix="${CATKIN_DEVEL_PREFIX}"
)

add_library(libav_wrap src/wrap_lib.cc)
SET(LIBAV_LIBRARY_DEST ${CATKIN_DEVEL_PREFIX}/lib)
SET(LIBAV_INCLUDE_DEST ${CATKIN_DEVEL_PREFIX}/include)

SET(LIBAV_LIBS ${LIBAV_LIBRARY_DEST}/libavcodec.so
  ${LIBAV_LIBRARY_DEST}/libavdevice.so ${LIBAV_LIBRARY_DEST}/libavfilter.so
  ${LIBAV_LIBRARY_DEST}/libavformat.so ${LIBAV_LIBRARY_DEST}/libavresample.so
  ${LIBAV_LIBRARY_DEST}/libavutil.so ${LIBAV_LIBRARY_DEST}/libswscale.so
)

SET(LIBAV_INCLUDE_DIRS ${LIBAV_INCLUDE_DEST}/libavcodec
  ${LIBAV_INCLUDE_DEST}/libavdevice ${LIBAV_INCLUDE_DEST}/libavfilter
  ${LIBAV_INCLUDE_DEST}/libavformat ${LIBAV_INCLUDE_DEST}/libavresample
  ${LIBAV_INCLUDE_DEST}/libavutil ${LIBAV_INCLUDE_DEST}/libswscale
)

set (LIBS_DEREF "")
foreach (_file ${LIBAV_LIBS})
    get_filename_component(_resolvedFile "${_file}" REALPATH)
    list (APPEND LIBS_DEREF "${_resolvedFile}")
endforeach()

target_link_libraries(libav_wrap ${LIBAV_LIBS})
add_dependencies(libav_wrap libav_trunk)

install(TARGETS libav_wrap
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY ${LIBAV_INCLUDE_DIRS}
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(FILES ${LIBS_DEREF}
  DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
)
