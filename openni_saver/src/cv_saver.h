#ifndef CV_SAVER
#define CV_SAVER

#include <sensor_msgs/Image.h>
#include <string>
#include <opencv2/opencv.hpp>

namespace cv_saver {

    cv::Mat lastDepth;
    cv::Mat lastRGB;
    bool gotDepth;
    bool gotRGB;
    std::string impath;
	long int start;
	void init_saver(const std::string&);
	void image_callback(const sensor_msgs::Image::ConstPtr& msg);
    void synch_callback(const sensor_msgs::Image::ConstPtr& msg);
	
}
#endif
