#include "openni_image_saver.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include "libav_compressor/CompressionService.h"
#include "openni_saver/LoggingService.h"
#include <iomanip>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

openni_image_saver::openni_image_saver(ros::ServiceClient& client, int video_length, std::string bag_folder) :
    client(client), video_length(1000*video_length), current_folder(""), parent_folder("")
{
    boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_duration duration(time.time_of_day());
    start = 0;
    recording = video_length > 0;
    at_waypoint = false;
    counter = 1;
    waypoint = 0;
    with_compression = true; // should be argument
    
    if (video_length > 0) { // create bag folder
        /*boost::filesystem::path dir(bag_folder);
        if(bag_folder.empty() || !boost::filesystem::create_directory(dir)) {
            std::cout << bag_folder << std::endl;
	        ROS_ERROR("Failed to create new parent directory.");
        }*/
        parent_folder = bag_folder;
        start_recording("");
    }
}

void openni_image_saver::image_callback(const sensor_msgs::Image::ConstPtr& depth_msg,
                                        const sensor_msgs::Image::ConstPtr& rgb_msg)
{
    if (!recording) {
        return;
    }
    
    int depth_sec = depth_msg->header.stamp.sec; // get message sec timestamp
    int depth_nsec = depth_msg->header.stamp.nsec; // get message nanosec timestamp
    int rgb_sec = rgb_msg->header.stamp.sec; // get message sec timestamp
    int rgb_nsec = rgb_msg->header.stamp.nsec; // get message nanosec timestamp
    
    ROS_INFO("Depth image timestamp: %d.%d, RGB image timestamp: %d.%d", depth_sec, depth_nsec, rgb_sec, rgb_nsec);
    boost::shared_ptr<sensor_msgs::Image> depth_tracked_object;
	cv_bridge::CvImageConstPtr depth_cv_img_boost_ptr;
	try {
		depth_cv_img_boost_ptr = cv_bridge::toCvShare(*depth_msg, depth_tracked_object);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	boost::shared_ptr<sensor_msgs::Image> rgb_tracked_object;
	cv_bridge::CvImageConstPtr rgb_cv_img_boost_ptr;
	try {
		rgb_cv_img_boost_ptr = cv_bridge::toCvShare(*rgb_msg, rgb_tracked_object);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
    // use lossless png compression
    std::vector<int> compression;
    compression.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression.push_back(0); // no compression
    
    char buffer[250];
    // save images with identifier, seconds and nanosec timestamps
    /*sprintf(buffer, "%s/rgb%06ld-%010d-%010d.png", current_folder.c_str(), counter, rgb_sec, rgb_nsec);
    cv::imwrite(buffer, rgb_cv_img_boost_ptr->image, compression);
    
    sprintf(buffer, "%s/depth%06ld-%010d-%010d.png", current_folder.c_str(), counter, depth_sec, depth_nsec);
    cv::imwrite(buffer, depth_cv_img_boost_ptr->image, compression);*/
    
    sprintf(buffer, "%s/rgb%06ld.png", current_folder.c_str(), counter);
    cv::imwrite(buffer, rgb_cv_img_boost_ptr->image, compression);
    
    sprintf(buffer, "%s/depth%06ld.png", current_folder.c_str(), counter);
    cv::imwrite(buffer, depth_cv_img_boost_ptr->image, compression);
    
    timestamps << "rgb";
    timestamps << std::setfill('0') << std::setw(6) << counter; // temporary
    timestamps << "-"; // temporary
    timestamps << std::setfill('0') << std::setw(10) << rgb_sec;
    timestamps << "-";
    timestamps << std::setfill('0') << std::setw(10) << rgb_nsec << std::endl;
    
    timestamps << "depth";
    timestamps << std::setfill('0') << std::setw(6) << counter; // temporary
    timestamps << "-"; // temporary
    timestamps << std::setfill('0') << std::setw(10) << depth_sec;
    timestamps << "-";
    timestamps << std::setfill('0') << std::setw(10) << depth_nsec << std::endl;
    
    ++counter;
    
	int delta;
	if (start == 0) {
	    start = get_time();
	    delta = 0;
	}
	else {
	    delta = get_time() - start;
	}
	
	if (!at_waypoint && video_length > 0 && delta > video_length) {
	    stop_recording();
	    start_recording("");
	    start = 0;
	}
}

int openni_image_saver::get_time()
{
    boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_duration duration(time.time_of_day());
	return duration.total_milliseconds();
}

void openni_image_saver::compress_folder()
{
    // launch a new online compressor instance
    libav_compressor::CompressionService srv;
    srv.request.folder = current_folder;
    if (!client.call(srv)) {
        ROS_ERROR("Failed to call compression service, possibly because of faulty paths.");
    }
}

bool openni_image_saver::stop_recording()
{
    recording = false;
    timestamps.close();
    if (with_compression) {
        compress_folder();
    }
    return true;
}

bool openni_image_saver::start_recording(const std::string& folder)
{
    if (parent_folder.empty()) {
        return false;
    }
    if (folder.empty()) {
        current_folder = parent_folder + "/" + boost::lexical_cast<std::string>(waypoint);
    }
    else {
        current_folder = parent_folder + "/" + folder;
    }
    boost::filesystem::path dir(current_folder);
    if(!boost::filesystem::create_directory(dir)) {
	    ROS_ERROR("Failed to create new current directory.");
    }
    counter = 1;
    recording = true;
    std::string filename = current_folder + "/time.txt";
    timestamps.open(filename.c_str());
    ++waypoint;
    return true;
}

bool openni_image_saver::logging_service(openni_saver::LoggingService::Request& req,
                                         openni_saver::LoggingService::Response& res)
{
    if (req.action == "start") { // start recording at a new waypoint
        if (video_length > 0) {
            stop_recording();
        }
        if (!start_recording(req.folder)) {
            res.success = false;
            return false;
        }
        res.success = true;
        at_waypoint = true;
    }
    else if (req.action == "stop") { // stop recording at waypoint
        stop_recording();
        if (video_length > 0) {
            if (!start_recording("")) { // timestamp would be better
                res.success = false;
                return false;
            }
        }
        start = 0;
        res.success = true;
        at_waypoint = false;
    }
    else if (req.action == "new") { // new patrol run
        recording = false;
        waypoint = 0;
        at_waypoint = false;
        start = 0;
        timestamps.close(); // assuming you can close it twice
        parent_folder = req.folder;
        boost::filesystem::path dir(parent_folder);
	    if(!boost::filesystem::create_directory(dir)) {
		    ROS_ERROR("Failed to create new parent directory.");
	    }
        res.success = true;
    }
    else {
        res.success = false;
    }
    return true;
}
