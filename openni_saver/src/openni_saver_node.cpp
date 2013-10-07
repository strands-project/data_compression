#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include "openni_image_saver.h"
#include "libav_compressor/CompressionService.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "openni_saver_node");
	ros::NodeHandle n;
	
	if (!n.hasParam("/openni_saver_node/camera_topic")) {
        ROS_INFO("You have to provide the camera_topic parameter!");
        return -1;
    }
    std::string camera_topic;
    n.getParam("/openni_saver_node/camera_topic", camera_topic);
    
    if (!n.hasParam("/openni_saver_node/bag_folder")) {
        ROS_INFO("You have to provide the bag_folder parameter!");
        return -1;
    }
    std::string bag_folder;
    n.getParam("/openni_saver_node/bag_folder", bag_folder);
    
    if (!n.hasParam("/openni_saver_node/video_length")) {
        ROS_INFO("You have to provide the video_length parameter!");
        return -1;
    }
    int video_length;
    n.getParam("/openni_saver_node/video_length", video_length);
    
    ros::ServiceClient client = n.serviceClient<libav_compressor::CompressionService>("compression_service");
    openni_image_saver saver(client, video_length, bag_folder);

    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, camera_topic + "/depth/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, camera_topic + "/rgb/image_color", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, rgb_sub);
    sync.registerCallback(&openni_image_saver::image_callback, &saver);
    ros::ServiceServer service = n.advertiseService("logging_service", &openni_image_saver::logging_service, &saver);
    ros::spin();
    
    return 0;
}
