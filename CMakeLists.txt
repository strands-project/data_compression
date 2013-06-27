################### Minimum Declarations

cmake_minimum_required(VERSION 2.4.6)

set(CMAKE_CXX_FLAGS "-g -fPIC -std=c++0x")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

link_directories(${LIBRARY_OUTPUT_PATH})

#################### Link VTK

#set(VTK_DIR "/usr/lib/vtk-5.8")
#find_package(VTK REQUIRED)
#include(${VTK_USE_FILE})

################### Include directories

include_directories(/opt/ros/groovy/include/opencv2)
link_directories(/opt/ros/groovy/lib)

include_directories(/home/nbore/Workspace/ffopencv2/libav/include)

link_directories(/home/nbore/Workspace/ffopencv2/libav/lib)
link_directories(/home/nbore/Workspace/ffopencv2/libav/bin)

#################### Source files

add_library(video_encoder src/video_encoder.cpp)
add_library(video_decoder src/video_decoder.cpp)
add_library(rgb_encoder src/rgb_encoder.cpp)
add_library(rgb_decoder src/rgb_decoder.cpp)
add_library(depth_encoder src/depth_encoder.cpp)
add_library(depth_decoder src/depth_decoder.cpp)
add_executable(test src/test2.cpp)
add_executable(avconv_test src/avconv_test.cpp)

#################### Link libraries

target_link_libraries(video_encoder opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)

target_link_libraries(video_encoder avfilter avcodec avutil avformat swscale)

target_link_libraries(video_decoder opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)

target_link_libraries(video_decoder avfilter avcodec avutil avformat swscale)

target_link_libraries(rgb_encoder video_encoder)
target_link_libraries(rgb_decoder video_decoder)
target_link_libraries(depth_encoder video_encoder)
target_link_libraries(depth_decoder video_decoder)

target_link_libraries(test rgb_encoder rgb_decoder depth_encoder depth_decoder)

target_link_libraries(test opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)

target_link_libraries(avconv_test opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_nonfree opencv_photo opencv_ts opencv_videostab opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_stitching opencv_video)