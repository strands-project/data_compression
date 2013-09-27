#ifndef OPENNI_IMAGE_SAVER_H
#define OPENNI_IMAGE_SAVER_H

#include <sensor_msgs/Image.h>
#include <string>
#include <opencv2/opencv.hpp>
#include "openni_saver/StartStopRecording.h"

class openni_image_saver {
private:
    bool recording;
    long int start;
    long int counter;
    std::string current_folder;
public:
    void image_callback(const sensor_msgs::Image::ConstPtr& depth_msg,
                        const sensor_msgs::Image::ConstPtr& rgb_msg);
    bool start_stop_recording(openni_saver::StartStopRecording::Request& req,
                              openni_saver::StartStopRecording::Response& res);
    openni_image_saver(const std::string& current_folder);
};

#endif // OPENNI_IMAGE_SAVER_H
