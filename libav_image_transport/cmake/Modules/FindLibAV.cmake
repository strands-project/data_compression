# Module for locating libav.
#
# Customizable variables:
#   LIBAV_ROOT_DIR
#     Specifies libav's root directory.
#
# Read-only variables:
#   LIBAV_FOUND
#     Indicates whether the library has been found.
#
#   LIBAV_INCLUDE_DIRS
#      Specifies libav's include directory.
#
#   LIBAV_LIBRARIES
#     Specifies libav libraries that should be passed to target_link_libararies.
#
#   LIBAV_<COMPONENT>_LIBRARIES
#     Specifies the libraries of a specific <COMPONENT>.
#
#   LIBAV_<COMPONENT>_FOUND
#     Indicates whether the specified <COMPONENT> was found.
#
#
# Copyright (c) 2012 Sergiu Dotenco
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTLIBAVLAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

INCLUDE (FindPackageHandleStandardArgs)

IF (CMAKE_VERSION VERSION_GREATER 2.8.7)
  SET (_LIBAV_CHECK_COMPONENTS FALSE)
ELSE (CMAKE_VERSION VERSION_GREATER 2.8.7)
  SET (_LIBAV_CHECK_COMPONENTS TRUE)
ENDIF (CMAKE_VERSION VERSION_GREATER 2.8.7)

FIND_PATH (LIBAV_ROOT_DIR
  NAMES include/libavcodec/avcodec.h
        include/libavdevice/avdevice.h
        include/libavfilter/avfilter.h
        include/libavformat/avformat.h
        include/libavutil/avutil.h
        include/libswscale/swscale.h
  PATHS ENV LIBAVROOT
  DOC "libav root directory")

FIND_PATH (LIBAV_INCLUDE_DIR
  NAMES libavcodec/avcodec.h
        libavdevice/avdevice.h
        libavfilter/avfilter.h
        libavformat/avformat.h
        libavutil/avutil.h
        libswscale/swscale.h
  HINTS ${LIBAV_ROOT_DIR}
  PATH_SUFFIXES include
  DOC "libav include directory")

if (NOT LibAV_FIND_COMPONENTS)
  set (LibAV_FIND_COMPONENTS avcodec avdevice avfilter avformat avutil swscale)
endif (NOT LibAV_FIND_COMPONENTS)

FOREACH (_LIBAV_COMPONENT ${LibAV_FIND_COMPONENTS})
  STRING (TOUPPER ${_LIBAV_COMPONENT} _LIBAV_COMPONENT_UPPER)
  SET (_LIBAV_LIBRARY_BASE LIBAV_${_LIBAV_COMPONENT_UPPER}_LIBRARY)

  FIND_LIBRARY (${_LIBAV_LIBRARY_BASE}
    NAMES ${_LIBAV_COMPONENT}
    HINTS ${LIBAV_ROOT_DIR}
    PATH_SUFFIXES bin lib
    DOC "libav ${_LIBAV_COMPONENT} library")

  MARK_AS_ADVANCED (${_LIBAV_LIBRARY_BASE})

  SET (LIBAV_${_LIBAV_COMPONENT_UPPER}_FOUND TRUE)
  SET (LibAV_${_LIBAV_COMPONENT}_FOUND ${LIBAV_${_LIBAV_COMPONENT_UPPER}_FOUND})

  IF (${_LIBAV_LIBRARY_BASE})
    # setup the LIBAV_<COMPONENT>_LIBRARIES variable
    SET (LIBAV_${_LIBAV_COMPONENT_UPPER}_LIBRARIES ${${_LIBAV_LIBRARY_BASE}})
    LIST (APPEND LIBAV_LIBRARIES ${LIBAV_${_LIBAV_COMPONENT_UPPER}_LIBRARIES})
  ELSE (${_LIBAV_LIBRARY_BASE})
    LIST (APPEND _LIBAV_MISSING_LIBRARIES ${_LIBAV_LIBRARY_BASE})
  ENDIF (${_LIBAV_LIBRARY_BASE})
ENDFOREACH (_LIBAV_COMPONENT ${LibAV_FIND_COMPONENTS})

SET (LIBAV_INCLUDE_DIRS ${LIBAV_INCLUDE_DIR})

IF (DEFINED _LIBAV_MISSING_COMPONENTS AND _LIBAV_CHECK_COMPONENTS)
  IF (NOT LibAV_FIND_QUIETLY)
    MESSAGE (STATUS "One or more libav components were not found:")
    # Display missing components indented, each on a separate line
    FOREACH (_LIBAV_MISSING_COMPONENT ${_LIBAV_MISSING_COMPONENTS})
      MESSAGE (STATUS "  " ${_LIBAV_MISSING_COMPONENT})
    ENDFOREACH (_LIBAV_MISSING_COMPONENT ${_LIBAV_MISSING_COMPONENTS})
  ENDIF (NOT LibAV_FIND_QUIETLY)
ENDIF (DEFINED _LIBAV_MISSING_COMPONENTS AND _LIBAV_CHECK_COMPONENTS)

# Determine library's version

FIND_PROGRAM (LIBAV_AVCONV_EXECUTABLE NAMES avconv
  HINTS ${LIBAV_ROOT_DIR}
  PATH_SUFFIXES bin
  DOC "avconv executable")

IF (LIBAV_AVCONV_EXECUTABLE)
  EXECUTE_PROCESS (COMMAND ${LIBAV_AVCONV_EXECUTABLE} -version
    OUTPUT_VARIABLE _LIBAV_AVCONV_OUTPUT ERROR_QUIET)

  STRING (REGEX REPLACE ".*avconv[ \t]+v?([0-9]+\\.[0-9]+).*" "\\1"
    LIBAV_VERSION "${_LIBAV_AVCONV_OUTPUT}")
  STRING (REGEX REPLACE "([0-9]+)\\.([0-9]+)" "\\1"
    LIBAV_VERSION_MAJOR "${LIBAV_VERSION}")
  STRING (REGEX REPLACE "([0-9]+)\\.([0-9]+)" "\\2"
    LIBAV_VERSION_MINOR "${LIBAV_VERSION}")

  IF (LIBAV_VERSION_MAJOR EQUAL 0)
    STRING (REGEX REPLACE ".*avconv[ \t]+([0-9]+\\.[0-9]+\\.[0-9]+).*" "\\1"
      LIBAV_VERSION "${_LIBAV_AVCONV_OUTPUT}")
    STRING (REGEX REPLACE "([0-9]+)\\.([0-9]+)\\.([0-9]+)" "\\2"
      LIBAV_VERSION_MAJOR "${LIBAV_VERSION}")
    STRING (REGEX REPLACE "([0-9]+)\\.([0-9]+)\\.([0-9]+)" "\\3"
      LIBAV_VERSION_MINOR "${LIBAV_VERSION}")
    SET (LIBAV_VERSION "${LIBAV_VERSION_MAJOR}.${LIBAV_VERSION_MINOR}")
  ENDIF (LIBAV_VERSION_MAJOR EQUAL 0)

  SET (LIBAV_VERSION_COMPONENTS 2)
ENDIF (LIBAV_AVCONV_EXECUTABLE)

MARK_AS_ADVANCED (LIBAV_INCLUDE_DIR)

IF (NOT _LIBAV_CHECK_COMPONENTS)
 SET (_LIBAV_FPHSA_ADDITIONAL_ARGS HANDLE_COMPONENTS)
ENDIF (NOT _LIBAV_CHECK_COMPONENTS)

message("-- libav ${LIBAV_VERSION}")

FIND_PACKAGE_HANDLE_STANDARD_ARGS (LibAV REQUIRED_VARS LIBAV_ROOT_DIR
  LIBAV_INCLUDE_DIR ${_LIBAV_MISSING_LIBRARIES} VERSION_VAR LIBAV_VERSION
  ${_LIBAV_FPHSA_ADDITIONAL_ARGS})
