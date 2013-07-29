#include "cv_saver.h"
#include <ros/ros.h>
#include <iostream>

int main(int argc, char** argv)
{

	//cv_saver saver;
	ros::init(argc, argv, "depth_saver_node");
	ros::NodeHandle n;
    if (!n.hasParam("/depth_saver_node/image_folder")) {
        return -1;
    }
    std::string image_folder;
    n.getParam("/depth_saver_node/image_folder", image_folder);
    cv_saver::init_saver(image_folder);
    //ros::Duration(0.5).sleep();
	ros::Subscriber sub = n.subscribe("camera/depth/image_raw", 2, &cv_saver::image_callback);
	ros::spin();

	return 0;
}
