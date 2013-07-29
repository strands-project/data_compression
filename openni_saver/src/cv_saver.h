#ifndef CV_SAVER
#define CV_SAVER

#include <sensor_msgs/Image.h>

namespace cv_saver {

	long int start;
	void init_time();
	void image_callback(const sensor_msgs::Image::ConstPtr& msg);
	
}
#endif