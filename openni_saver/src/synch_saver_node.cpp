#include "cv_saver.h"
#include <ros/ros.h>
#include <iostream>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "synch_saver_node");
	ros::NodeHandle n;
    if (!n.hasParam("/synch_saver_node/image_folder")) {
        std::cout << "You have to provide the image_folder parameter!" << std::endl;
        return -1;
    }
    std::string image_folder;
    n.getParam("/synch_saver_node/image_folder", image_folder);
    cv_saver::init_saver(image_folder);
    //ros::Duration(0.5).sleep();
	ros::Subscriber depthSub = n.subscribe("camera/depth/image_raw", 10, &cv_saver::synch_callback);
    ros::Subscriber rgbSub = n.subscribe("camera/rgb/image_color", 10, &cv_saver::synch_callback);
	ros::spin();

	return 0;
}
