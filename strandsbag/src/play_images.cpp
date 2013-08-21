#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
//#include <std_msgs/Header.h>
#include <rosgraph_msgs/Clock.h>
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
    header_depth.frame_id = "/camera_depth_frame"; // should probably be a parameter
    header_rgb.frame_id = "/camera_rgb_frame"; // should probably be a parameter
    
    image_depth = cv_bridge::CvImage(header_depth, "16UC1", temp_depth);
    image_rgb = cv_bridge::CvImage(header_rgb, "8UC3", temp_rgb);
    /*image_depth.header = header_depth;
    image_depth.encoding = "16UC1";
    image_depth.image = temp_depth.clone();
    
    image_rgb.header = header_rgb;
    image_rgb.encoding = "8UC3";
    image_rgb.image = temp_rgb.clone();*/
    
    /*if (remove(depth_path.c_str()) != 0) {
        ROS_ERROR("Error deleting depth file.");
    }
    if (remove(rgb_path.c_str()) != 0) {
        ROS_ERROR("Error deleting rgb file.");
    }*/
}

void time_callback(const rosgraph_msgs::ClockPtr& msg)
{
    ros::Time clock = msg->clock;
    std::cout << "now i received a clock: " << clock << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "play_images");
	ros::NodeHandle n;
    if (!n.hasParam("/play_images/image_folder")) {
        ROS_ERROR("Could not find parameter image_folder.");
        return -1;
    }
    std::string image_folder;
    n.getParam("/play_images/image_folder", image_folder);
	ros::Publisher depth_pub = n.advertise<sensor_msgs::Image>("/player/depth", 10);
	ros::Publisher rgb_pub = n.advertise<sensor_msgs::Image>("/player/rgb", 10);
	ros::Subscriber time_sub = n.subscribe("/clock", 1, &time_callback);
	//n.subscribe("camera/depth/image_raw", 2, &cv_saver::image_callback);
	
	ros::Rate loop_rate(30);

    /**
    * A count of how many messages we have sent. This is used to create
    * a unique string for each message.
    */
    int count = 0;
    std::vector<std::string> depth_files;
    std::vector<std::string> rgb_files;
    update_files(image_folder, depth_files, rgb_files);
    cv::Mat temp_depth(480, 640, CV_16UC1);
    cv::Mat temp_rgb(480, 640, CV_8UC3);
    cv_bridge::CvImage image_depth;
    cv_bridge::CvImage image_rgb;
    read_images(image_depth, image_rgb, temp_depth, temp_rgb, image_folder, depth_files.back(), rgb_files.back());
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
        
        // if we have too few images we should send a message to decompress more
        if (depth_files.size() < 60 || rgb_files.size() < 60) {
            //update_files(image_folder, depth_files, rgb_files);
            if (depth_files.empty() || rgb_files.empty()) {
                ROS_INFO("There are no more images to show, stopping.");
                break;
            }
        }
        
        read_images(image_depth, image_rgb, temp_depth, temp_rgb, image_folder, depth_files.back(), rgb_files.back());
        depth_files.pop_back();
        rgb_files.pop_back();

        loop_rate.sleep();
        ++count;
    }

	
	return 0;
}
