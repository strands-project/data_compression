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
    bool recording;
    bool at_waypoint;
    int video_length;
    long int start;
    long int counter;
    int waypoint;
    std::string current_folder;
    std::string parent_folder;
    bool with_compression;
    void compress_folder();
    ros::ServiceClient& client;
    std::ofstream timestamps;
    bool start_recording(const std::string& folder);
    bool stop_recording();
    int get_time();
public:
    void image_callback(const sensor_msgs::Image::ConstPtr& depth_msg,
                        const sensor_msgs::Image::ConstPtr& rgb_msg);
    bool logging_service(openni_saver::LoggingService::Request& req,
                         openni_saver::LoggingService::Response& res);
    openni_image_saver(ros::ServiceClient& client, int video_length = 0, std::string bag_folder = "");
};

#endif // OPENNI_IMAGE_SAVER_H
