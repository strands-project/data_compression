#ifndef CV_SAVER
#define CV_SAVER

#include <sensor_msgs/Image.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <list>
#include "openni_saver/StartStopRecording.h"

// TODO: turn this into a class!
namespace cv_saver {

    std::list<cv::Mat> depths; // depth image stack
    std::list<cv::Mat> rgbs; // rgb image stack
    std::list<int> depth_secs; // sec timestamp stacks
    std::list<int> rgb_secs;
    std::list<int> depth_nsecs; // nanosec timestamp stacks
    std::list<int> rgb_nsecs;
    std::string impath; // image folder
	long int start; // start millisec
	bool recording;
	void init_saver(const std::string&); // initalize variable
	void image_callback(const sensor_msgs::Image::ConstPtr& msg); // save depth or rgb
    void synch_callback(const sensor_msgs::Image::ConstPtr& msg); // save depth and rgb
    bool start_stop_recording(openni_saver::StartStopRecording::Request& req,
                              openni_saver::StartStopRecording::Response& res);
	
}
#endif
