#ifndef BAG_PLAYER_H
#define BAG_PLAYER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>

class bag_player {
private:
    ros::NodeHandle& n;
    ros::Publisher& depth_pub;
    ros::Publisher& rgb_pub;
    std::string dirname;
    std::vector<std::string> depth_files;
    std::vector<std::string> rgb_files;
    cv::Mat temp_depth;
    cv::Mat temp_rgb;
    cv_bridge::CvImage image_depth;
    cv_bridge::CvImage image_rgb;
    ros::Timer depth_timer;
    ros::Timer rgb_timer;
    
    ros::Time read_depth();
    ros::Time read_rgb();
    void update_files();
public:
    void publish_depth(const ros::TimerEvent& e);
    void publish_rgb(const ros::TimerEvent& e);
    bag_player(ros::NodeHandle& n, ros::Publisher& depth_pub, ros::Publisher& rgb_pub, const std::string& dirname);
};

#endif
