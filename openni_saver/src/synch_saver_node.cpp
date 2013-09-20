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
    if (!n.hasParam("/synch_saver_node/camera_topic")) {
        std::cout << "You have to provide the camera_topic parameter!" << std::endl;
        return -1;
    }
    std::string camera_topic;
    n.getParam("/synch_saver_node/camera_topic", camera_topic);
    cv_saver::init_saver(image_folder);
    std::cout << camera_topic + "/depth/image_raw" << std::endl;
	ros::Subscriber depthSub = n.subscribe(camera_topic + "/depth/image_raw", 10, &cv_saver::synch_callback);
    ros::Subscriber rgbSub = n.subscribe(camera_topic + "/rgb/image_color", 10, &cv_saver::synch_callback);
	ros::spin();

	return 0;
}
