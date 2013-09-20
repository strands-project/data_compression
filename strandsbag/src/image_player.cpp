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
    if (!n.hasParam("/image_player/camera_topic")) {
        ROS_ERROR("Could not find parameter camera_topic.");
        return -1;
    }
    std::string camera_topic;
    n.getParam("/image_player/camera_topic", camera_topic);
	ros::Publisher depth_pub = n.advertise<sensor_msgs::Image>(camera_topic + "/depth/image_raw", 10);
	ros::Publisher rgb_pub = n.advertise<sensor_msgs::Image>(camera_topic + "/rgb/image_color", 10);
    ros::Duration(0.18f).sleep();
    // the magic happens in the bag_player
    bag_player player(n, depth_pub, rgb_pub, image_folder);
    ros::spin();
	
	return 0;
}
