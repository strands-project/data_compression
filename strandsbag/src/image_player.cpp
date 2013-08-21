#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include "bag_player.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_player");
	ros::NodeHandle n;
    if (!n.hasParam("/image_player/image_folder")) {
        ROS_ERROR("Could not find parameter image_folder.");
        return -1;
    }
    std::string image_folder;
    n.getParam("/image_player/image_folder", image_folder);
	ros::Publisher depth_pub = n.advertise<sensor_msgs::Image>("/bag_player/depth", 10);
	ros::Publisher rgb_pub = n.advertise<sensor_msgs::Image>("/bag_player/rgb", 10);
    ros::Duration(0.18f).sleep();
    bag_player player(n, depth_pub, rgb_pub, image_folder);
    ros::spin();
	
	return 0;
}
