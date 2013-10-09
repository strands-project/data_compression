#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include "folder_player.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_player_node");
	ros::NodeHandle n;
	
	// the folder where the videos and ros bag are
    if (!n.hasParam("/image_player_node/bag_folder")) {
        ROS_ERROR("Could not find parameter bag_folder.");
        return -1;
    }
    std::string bag_folder;
    n.getParam("/image_player_node/bag_folder", bag_folder);
    
    // topic of the depth and rgb images
    if (!n.hasParam("/image_player_node/camera_topic")) {
        ROS_ERROR("Could not find parameter camera_topic.");
        return -1;
    }
    std::string camera_topic;
    n.getParam("/image_player_node/camera_topic", camera_topic);
    
	ros::Publisher depth_pub = n.advertise<sensor_msgs::Image>(camera_topic + "/depth/image_raw", 10);
	ros::Publisher rgb_pub = n.advertise<sensor_msgs::Image>(camera_topic + "/rgb/image_raw", 10);
    ros::Duration(0.18f).sleep();
    // the magic happens in the folder_player
    folder_player player(n, depth_pub, rgb_pub, bag_folder, camera_topic);
    ros::spin();
	
	return 0;
}
