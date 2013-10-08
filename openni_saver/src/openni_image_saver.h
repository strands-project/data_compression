#ifndef OPENNI_IMAGE_SAVER_H
#define OPENNI_IMAGE_SAVER_H

#include <sensor_msgs/Image.h>
#include <string>
#include <opencv2/opencv.hpp>
#include "openni_saver/LoggingService.h"
#include <ros/ros.h>
#include <fstream>

class openni_image_saver {
private:
    bool recording; // save image files?
    bool at_waypoint; // at waypoint?
    int video_length; // length of the stored videos, 0 if no continuous recording
    long int start; // first time of video file
    long int counter; // image number
    int waypoint; // waypoint number
    std::string current_folder; // recording folder
    std::string parent_folder; // bag folder
    bool with_compression; // with compression?
    void compress_folder(); // compress a folder by calling compression service with folder name
    ros::ServiceClient& client; // to be able to call compression service
    std::ofstream timestamps; // current timestamps file
    bool start_recording(const std::string& folder); // start recording
    bool stop_recording(); // stop recording
    int get_time();
    std::string ros_time_string(); // current ros time as string
public:
    // callback for saving depth and rgb image
    void image_callback(const sensor_msgs::Image::ConstPtr& depth_msg,
                        const sensor_msgs::Image::ConstPtr& rgb_msg);
    // call logging server with action and folder name
    bool logging_service(openni_saver::LoggingService::Request& req,
                         openni_saver::LoggingService::Response& res);
    // set video_length 0 if not continuous recording
    openni_image_saver(ros::ServiceClient& client, int video_length = 0, std::string bag_folder = "");
};

#endif // OPENNI_IMAGE_SAVER_H
