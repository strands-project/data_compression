#include "cv_saver.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{

	//cv_saver saver;
	ros::init(argc, argv, "rgb_saver_node");
	ros::NodeHandle n;
    if (!n.hasParam("image_folder")) {
        return -1;
    }
    std::string image_folder;
    n.getParam("image_folder", image_folder);
    cv_saver::init_saver(image_folder);
	ros::Subscriber sub = n.subscribe("camera/rgb/image_color", 2, &cv_saver::image_callback);
	ros::spin();

	return 0;
}
