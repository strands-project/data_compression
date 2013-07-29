#ifndef CV_SAVER
#define CV_SAVER

#include <sensor_msgs/Image.h>
#include <string>

namespace cv_saver {

    std::string impath;
	long int start;
	void init_saver(const std::string&);
	void image_callback(const sensor_msgs::Image::ConstPtr& msg);
	
}
#endif
