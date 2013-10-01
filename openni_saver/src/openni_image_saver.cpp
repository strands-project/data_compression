#include "openni_image_saver.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include "libav_compressor/CompressionService.h"
#include "openni_saver/LoggingService.h"
#include <iomanip>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

openni_image_saver::openni_image_saver(ros::ServiceClient& client) :
    client(client), current_folder(""), parent_folder("")
{
    boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_duration duration(time.time_of_day());
    start = duration.total_milliseconds();
    recording = false;
    counter = 0;
    waypoint = 0;
    with_compression = true;
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
	
	/*boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_duration duration(time.time_of_day());
	int delta = duration.total_milliseconds() - start;*/
	
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
    timestamps << std::setfill ('0') << std::setw(6) << counter; // temporary
    timestamps << "-"; // temporary
    timestamps << std::setfill ('0') << std::setw(10) << rgb_sec;
    timestamps << "-";
    timestamps << std::setfill ('0') << std::setw(10) << rgb_nsec << std::endl;
    
    timestamps << "depth";
    timestamps << std::setfill ('0') << std::setw(6) << counter; // temporary
    timestamps << "-"; // temporary
    timestamps << std::setfill ('0') << std::setw(10) << depth_sec;
    timestamps << "-";
    timestamps << std::setfill ('0') << std::setw(10) << depth_nsec << std::endl;
    
    ++counter;
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

bool openni_image_saver::logging_service(openni_saver::LoggingService::Request& req,
                                         openni_saver::LoggingService::Response& res)
{
    if (req.action == "start") { // start recording at a new waypoint
        if (parent_folder.empty()) {
            res.success = false;
            return false;
        }
        if (req.folder.empty()) {
            current_folder = parent_folder + "/" + boost::lexical_cast<std::string>(waypoint);
        }
        else {
            current_folder = parent_folder + "/" + req.folder;
        }
        boost::filesystem::path dir(current_folder);
	    if(!boost::filesystem::create_directory(dir)) {
		    ROS_ERROR("Failed to create new current directory.");
	    }
        counter = 0;
        recording = true;
        res.success = true;
        std::string filename = current_folder + "/time.txt";
        timestamps.open(filename.c_str());
        ++waypoint;
    }
    else if (req.action == "stop") { // stop recording at waypoint
        recording = false;
        res.success = true;
        timestamps.close();
        if (with_compression) {
            compress_folder();
        }
    }
    else if (req.action == "new") { // new patrol run
        recording = false;
        waypoint = 0;
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
