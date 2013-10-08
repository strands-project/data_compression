#include "openni_image_saver.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include "libav_compressor/CompressionService.h"
#include "openni_saver/LoggingService.h"
#include <iomanip>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>

openni_image_saver::openni_image_saver(ros::ServiceClient& client, int video_length, std::string bag_folder) :
    client(client), video_length(1000*video_length), current_folder(""), parent_folder("")
{
    start = 0; // clock not initialized
    recording = video_length > 0; // if not constant recording, send in video_length = 0
    at_waypoint = false; // recording at waypoint
    counter = 1; // image number
    waypoint = 0; // waypoint number
    with_compression = true; // should be argument
    
    if (video_length > 0) { // if constant recording, start directly
        parent_folder = bag_folder;
        start_recording(ros_time_string());
    }
}

// gives current ros time as string
std::string openni_image_saver::ros_time_string()
{
    ros::Time now = ros::Time::now();
    std::stringstream s;
    s << now;
    return s.str();
}

// called by the synchronizer, always with depth + rgb
void openni_image_saver::image_callback(const sensor_msgs::Image::ConstPtr& depth_msg,
                                        const sensor_msgs::Image::ConstPtr& rgb_msg)
{
    if (!recording) { // if not recording don't save
        return;
    }
    
    int depth_sec = depth_msg->header.stamp.sec; // get message sec timestamp
    int depth_nsec = depth_msg->header.stamp.nsec; // get message nanosec timestamp
    int rgb_sec = rgb_msg->header.stamp.sec; // get message sec timestamp
    int rgb_nsec = rgb_msg->header.stamp.nsec; // get message nanosec timestamp
    
    //ROS_INFO("Depth image timestamp: %d.%d, RGB image timestamp: %d.%d", depth_sec, depth_nsec, rgb_sec, rgb_nsec);
    
    // convert message to opencv images for saving
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
    
    sprintf(buffer, "%s/rgb%06ld.png", current_folder.c_str(), counter);
    cv::imwrite(buffer, rgb_cv_img_boost_ptr->image, compression);
    
    sprintf(buffer, "%s/depth%06ld.png", current_folder.c_str(), counter);
    cv::imwrite(buffer, depth_cv_img_boost_ptr->image, compression);
    
    // write the timestamps to time.txt
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
	
	// if video_length exceeded, start new video
	if (!at_waypoint && video_length > 0 && delta > video_length) {
	    stop_recording();
	    start_recording(ros_time_string());
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
    // call online compressor to start new compression thread
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

// create new folder and timestamps file
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

// service function, call it with start, stop or new as action
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
            if (!start_recording(ros_time_string())) { // timestamp would be better
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
