#include "openni_image_saver.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/date_time/posix_time/posix_time_types.hpp>

openni_image_saver::openni_image_saver(const std::string& current_folder) :
    current_folder(current_folder)
{
    boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_duration duration(time.time_of_day());
    start = duration.total_milliseconds();
    recording = true;
    counter = 0;
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
    sprintf(buffer, "%s/rgb%06ld-%010d-%010d.png", current_folder.c_str(), counter, rgb_sec, rgb_nsec);
    cv::imwrite(buffer, rgb_cv_img_boost_ptr->image, compression);
    
    sprintf(buffer, "%s/depth%06ld-%010d-%010d.png", current_folder.c_str(), counter, depth_sec, depth_nsec);
    cv::imwrite(buffer, depth_cv_img_boost_ptr->image, compression);
    
    ++counter;
}

bool openni_image_saver::start_stop_recording(openni_saver::StartStopRecording::Request& req,
                                              openni_saver::StartStopRecording::Response& res)
{
    if (req.action == "start") {
        recording = true;
        res.success = true;
    }
    else if (req.action == "stop") {
        recording = false;
        res.success = true;
    }
    else {
        res.success = false;
    }
    return true;
}
