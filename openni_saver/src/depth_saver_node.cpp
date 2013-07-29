#include "cv_saver.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{

	//cv_saver saver;
	cv_saver::init_time();
	ros::init(argc, argv, "depth_saver_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("camera/depth/image_raw", 2, &cv_saver::image_callback);
	ros::spin();

	return 0;
}