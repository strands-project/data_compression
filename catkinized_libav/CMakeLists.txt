cmake_minimum_required(VERSION 2.8.3)
project(catkinized_libav)
 
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
 
find_package(catkin REQUIRED)
 
include(ExternalProject)
 
set(VERSION 2.4.1)
 
catkin_package(
#DEPENDS
#CATKIN_DEPENDS
#INCLUDE_DIRS include # libav is already in /devel/include
LIBRARIES libav_wrap
)

ExternalProject_Add(libav_trunk
  DOWNLOAD_COMMAND ""
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libav_trunk"
  CONFIGURE_COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/libav_trunk/configure" --enable-shared --enable-libx264 --enable-gpl --prefix="${CATKIN_DEVEL_PREFIX}"
)

add_library(libav_wrap src/wrap_lib.cc)
set(LIBAV_LIBRARY_DEST ${CATKIN_DEVEL_PREFIX}/lib)
set(LIBAV_INCLUDE_DEST ${CATKIN_DEVEL_PREFIX}/include)

set(LIBAV_COMPONENTS avcodec avdevice avfilter avformat avresample avutil swscale)

set (LIBAV_LIBS "")
set (LIBAV_INCLUDE_DIRS "")
foreach (_name ${LIBAV_COMPONENTS})
    #find_library(_library_file
    #  NAMES ${_name}
    #  PATHS ${LIBAV_LIBRARY_DEST}
    #  NO_DEFAULT_PATH
    #)
    #list (APPEND LIBAV_LIBS "${_library_file}")
    list (APPEND LIBAV_LIBS "${LIBAV_LIBRARY_DEST}/lib${_name}.so")
    list (APPEND LIBAV_INCLUDE_DIRS "${LIBAV_INCLUDE_DEST}/lib${_name}")
endforeach()

#target_link_libraries(libav_wrap -Wl,--whole-archive ${LIBAV_LIBS} -Wl,--no-whole-archive)
add_dependencies(libav_wrap libav_trunk)
target_link_libraries(libav_wrap ${LIBAV_LIBS})

install(TARGETS libav_wrap
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY ${LIBAV_INCLUDE_DIRS}
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

#install(DIRECTORY ${LIBAV_LIBRARY_DEST}/.
#  DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
#  EXPORT_LINK_INTERFACE_LIBRARIES
#)

foreach (_name ${LIBAV_COMPONENTS})
  install(DIRECTORY ${LIBAV_LIBRARY_DEST}/.
    RUNTIME DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
    FILES_MATCHING
    PATTERN "*${_name}*"
  )
endforeach()
