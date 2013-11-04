#ifndef FOLDER_PLAYER_H
#define FOLDER_PLAYER_H

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>
#include <string>
#include <utility>

class folder_player {
private:
    ros::NodeHandle& n; // We need the nodehandle to set timers
    ros::Publisher& depth_pub; // To publish depth
    ros::Publisher& rgb_pub; // Publish color
    std::string camera; // name of camera topic
    std::string dirname; // Image directory path
    cv::Mat temp_depth; // For reading from disk
    cv::Mat temp_rgb;
    cv_bridge::CvImage image_depth; // Intermediary format before publishing
    cv_bridge::CvImage image_rgb;
    ros::Timer depth_timer; // Sets timer to publish according to timestamp
    ros::Timer rgb_timer;
    
    std::vector<std::pair<std::string, double> > dirs; // folder to be played + first timestamp
    std::deque<int> depth_secs; // depth sec timestamps
    std::deque<int> depth_nsecs; // depth nanosec timestamps
    std::deque<int> rgb_secs; // rgb sec timestamps
    std::deque<int> rgb_nsecs; // rgb nanosec timestamps
    std::deque<std::string> depth_names; // depth filenames
    std::deque<std::string> rgb_names; // rgb filenames
    
    int dir_nbr;
    bool shutting_down;
    
    ros::Time read_depth(); // Reads the latest depth file from disk
    ros::Time read_rgb(); // Reads the latest RGB file from disk
    void read_new_dir(); // Updates the file lists
public:
    void publish_depth(const ros::TimerEvent& e); // Publishes the new depth image
    void publish_rgb(const ros::TimerEvent& e); // Publishes new rgb image
    folder_player(ros::NodeHandle& n, ros::Publisher& depth_pub, ros::Publisher& rgb_pub,
                  const std::string& dirname, const std::string& camera, const std::string& sub_folder);
};

#endif
