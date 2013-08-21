#include "bag_player.h"

#include <sensor_msgs/Image.h>
#include <dirent.h>
#include <iostream>
#include <string>

bag_player::bag_player(ros::NodeHandle& n, ros::Publisher& depth_pub, ros::Publisher& rgb_pub, const std::string& dirname) : n(n), depth_pub(depth_pub), rgb_pub(rgb_pub), dirname(dirname), temp_depth(480, 640, CV_16UC1), temp_rgb(480, 640, CV_8UC3)
{
    update_files();
    ros::Time t = read_depth();
    depth_timer = n.createTimer(t - ros::Time::now(), &bag_player::publish_depth, this, true);
    t = read_rgb();
    rgb_timer = n.createTimer(t - ros::Time::now(), &bag_player::publish_rgb, this, true);
}

void bag_player::update_files()
{
    depth_files.clear();
    rgb_files.clear();
    std::string depth_name("depth");
    std::string rgb_name("rgb");
    DIR* dir = opendir(dirname.c_str());
    if (dir == NULL) {
        ROS_ERROR("Can not read video directory.");
    }
    struct dirent* ent;
    while ((ent = readdir(dir)) != NULL) {
        std::string entry(ent->d_name);
        if (entry.compare(0, depth_name.size(), depth_name) == 0) {
	        depth_files.push_back(entry);
	    }
	    else if (entry.compare(0, rgb_name.size(), rgb_name) == 0) {
	        rgb_files.push_back(entry);
	    }
	    else {
	        continue;
	    }
    }
    closedir(dir);
    std::sort(depth_files.begin(), depth_files.end());
    std::reverse(depth_files.begin(), depth_files.end());
    std::sort(rgb_files.begin(), rgb_files.end());
    std::reverse(rgb_files.begin(), rgb_files.end());
}

ros::Time bag_player::read_depth()
{
    std::string depth_file = depth_files.back();
    depth_files.pop_back();
    std::string depth_path = dirname + "/" + depth_file;
    temp_depth = cv::imread(depth_path, -1);
    
    std::string depth_sec = depth_file.substr(12, 10);
    std::string depth_nsec = depth_file.substr(23, 10);
    
    std_msgs::Header header_depth;
    header_depth.stamp = ros::Time(atoi(depth_sec.c_str()), atoi(depth_nsec.c_str()));
    
    std::cout << "Depth time: " << header_depth.stamp << std::endl;
    
    header_depth.frame_id = "/camera_depth_frame"; // should probably be a parameter
    
    image_depth = cv_bridge::CvImage(header_depth, "16UC1", temp_depth);
    
    /*if (remove(depth_path.c_str()) != 0) {
        ROS_ERROR("Error deleting depth file.");
    }*/
    return header_depth.stamp;
}

ros::Time bag_player::read_rgb()
{
    std::string rgb_file = rgb_files.back();
    rgb_files.pop_back();
    std::string rgb_path = dirname + "/" + rgb_file;
    temp_rgb = cv::imread(rgb_path, -1);
    
    std::string rgb_sec = rgb_file.substr(10, 10);
    std::string rgb_nsec = rgb_file.substr(21, 10);
    
    std_msgs::Header header_rgb;
    header_rgb.stamp = ros::Time(atoi(rgb_sec.c_str()), atoi(rgb_nsec.c_str()));
    
    std::cout << "RGB time: " << header_rgb.stamp << std::endl;
    
    header_rgb.frame_id = "/camera_rgb_frame"; // should probably be a parameter
    
    image_rgb = cv_bridge::CvImage(header_rgb, "8UC3", temp_rgb);
    
    /*if (remove(rgb_path.c_str()) != 0) {
        ROS_ERROR("Error deleting rgb file.");
    }*/
    return header_rgb.stamp;
}

void bag_player::publish_depth(const ros::TimerEvent& e)
{
    depth_pub.publish(image_depth.toImageMsg());
    if (depth_files.size() == 0) {
        return;
    }
    ros::Time t = read_depth();
    depth_timer = n.createTimer(t - ros::Time::now(), &bag_player::publish_depth, this, true);
}

void bag_player::publish_rgb(const ros::TimerEvent& e)
{
    rgb_pub.publish(image_rgb.toImageMsg());
    if (rgb_files.size() == 0) {
        return;
    }
    ros::Time t = read_rgb();
    rgb_timer = n.createTimer(t - ros::Time::now(), &bag_player::publish_rgb, this, true);
}
