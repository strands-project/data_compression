#ifndef CV_SAVER
#define CV_SAVER

#include <sensor_msgs/Image.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <list>

namespace cv_saver {

    std::list<cv::Mat> depths;
    std::list<cv::Mat> rgbs;
    std::list<int> depth_secs;
    std::list<int> rgb_secs;
    std::list<int> depth_nsecs;
    std::list<int> rgb_nsecs;
    std::string impath;
	long int start;
	void init_saver(const std::string&);
	void image_callback(const sensor_msgs::Image::ConstPtr& msg);
    void synch_callback(const sensor_msgs::Image::ConstPtr& msg);
	
}
#endif
