#ifndef BAG_PLAYER_H
#define BAG_PLAYER_H

#include <ros/ros.h>
//#include <ros/time.h>
#include <rosgraph_msgs/Clock.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>

class bag_player {
private:
    ros::NodeHandle& n; // We need the nodehandle to set timers
    ros::Publisher& depth_pub; // To publish depth
    ros::Publisher& rgb_pub; // Publish color
    std::string dirname; // Image directory path
    std::vector<std::string> depth_files; // Depth filenames
    std::vector<std::string> rgb_files; // RGB filenames
    cv::Mat temp_depth; // For reading from disk
    cv::Mat temp_rgb;
    cv_bridge::CvImage image_depth; // Intermediary format before publishing
    cv_bridge::CvImage image_rgb;
    ros::Timer depth_timer; // Sets timer to publish according to timestamp
    ros::Timer rgb_timer;
    
    ros::Time read_depth(); // Reads the latest depth file from disk
    ros::Time read_rgb(); // Reads the latest RGB file from disk
    void update_files(); // Updates the file lists
public:
    void publish_depth(const ros::TimerEvent& e); // Publishes the new depth image
    void publish_rgb(const ros::TimerEvent& e); // Publishes new rgb image
    bag_player(ros::NodeHandle& n, ros::Publisher& depth_pub, ros::Publisher& rgb_pub, const std::string& dirname);
};

#endif
