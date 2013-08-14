#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
//#include <std_msgs/Header.h>
#include <vector>
#include <string>
#include <dirent.h>
#include <stdio.h>

void update_files(const std::string& dirname, std::vector<std::string>& depth_files, std::vector<std::string>& rgb_files)
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

void read_images(cv_bridge::CvImage& image_depth, cv_bridge::CvImage& image_rgb, cv::Mat& temp_depth, cv::Mat& temp_rgb, const std::string& dirname, const std::string& depth_file, const std::string& rgb_file)
{
    std::string depth_path = dirname + "/" + depth_file;
    std::string rgb_path = dirname + "/" + rgb_file;
    temp_depth = cv::imread(depth_path, -1);
    temp_rgb = cv::imread(rgb_path, -1);
    
    std_msgs::Header header_depth;
    std_msgs::Header header_rgb;
    header_depth.stamp = ros::Time().now();
    header_rgb.stamp = header_depth.stamp;
    header_depth.frame_id = "/camera_depth_frame";
    header_rgb.frame_id = "/camera_rgb_frame";
    
    image_depth = cv_bridge::CvImage(header_depth, "CV16UC1", temp_depth);
    image_rgb = cv_bridge::CvImage(header_rgb, "CV8UC1", temp_rgb);
    
    if (remove(depth_path.c_str()) != 0) {
        ROS_ERROR("Error deleting depth file.");
    }
    if (remove(rgb_path.c_str()) != 0) {
        ROS_ERROR("Error deleting rgb file.");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "play_images");
	ros::NodeHandle n;
    if (!n.hasParam("/depth_saver_node/video_folder")) {
        ROS_ERROR("Could not find parameter video_folder.");
        return -1;
    }
    std::string video_folder;
    n.getParam("/depth_saver_node/video_folder", video_folder);
	ros::Publisher depth_pub = n.advertise<sensor_msgs::Image>("/camera/depth", 1000);
	ros::Publisher rgb_pub = n.advertise<sensor_msgs::Image>("/camera/rgb", 1000);
	
	ros::Rate loop_rate(30);

    /**
    * A count of how many messages we have sent. This is used to create
    * a unique string for each message.
    */
    int count = 0;
    std::vector<std::string> depth_files;
    std::vector<std::string> rgb_files;
    update_files(video_folder, depth_files, rgb_files);
    cv::Mat temp_depth(480, 640, CV_16UC1);
    cv::Mat temp_rgb(480, 640, CV_8UC3);
    cv_bridge::CvImage image_depth;
    cv_bridge::CvImage image_rgb;
    read_images(image_depth, image_rgb, temp_depth, temp_rgb, video_folder, depth_files.back(), rgb_files.back());
    depth_files.pop_back();
    rgb_files.pop_back();
    
    while (ros::ok())
    {

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        depth_pub.publish(image_depth.toImageMsg());
        rgb_pub.publish(image_rgb.toImageMsg());

        ros::spinOnce();
        
        if (depth_files.size() < 60 || rgb_files.size() < 60) {
            update_files(video_folder, depth_files, rgb_files);
            if (depth_files.empty() || rgb_files.empty()) {
                ROS_INFO("There are no more images to show, stopping.");
                break;
            }
        }
        
        read_images(image_depth, image_rgb, temp_depth, temp_rgb, video_folder, depth_files.front(), rgb_files.front());

        loop_rate.sleep();
        ++count;
    }

	
	return 0;
}
