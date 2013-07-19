cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include(ExternalProject)

INSTALL(CODE "MESSAGE(\"make install\")")

ExternalProject_Add(libav_trunk
  PREFIX libav_trunk
#  GIT_REPOSITORY https://github.com/libav/libav.git
#  GIT_TAG v9.8
#  DOWNLOAD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libav_trunk"
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libav_trunk"
  CONFIGURE_COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/libav_trunk/configure" --enable-shared --enable-libx264 --enable-gpl --prefix="${CMAKE_INSTALL_PREFIX}"
)
