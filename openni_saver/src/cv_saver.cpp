#include "cv_saver.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <iostream>

namespace cv_saver {

	void init_saver(const std::string& path)
	{
        impath = path;
        gotDepth = false;
        gotRGB = false;
		boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration duration(time.time_of_day());
		start = duration.total_milliseconds();
	}

	void image_callback(const sensor_msgs::Image::ConstPtr& msg)
	{
	    std::cout << "Time lag = " << msg->header.stamp.sec << "." << msg->header.stamp.nsec << std::endl;
		boost::shared_ptr<sensor_msgs::Image> tracked_object;
		cv_bridge::CvImageConstPtr cv_img_boost_ptr;
		try {
			cv_img_boost_ptr = cv_bridge::toCvShare(*msg, tracked_object);
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
	    boost::posix_time::time_duration duration(time.time_of_day());
	    char buffer[250];
	    if (cv_img_boost_ptr->image.type() == CV_16UC1) {
	    	std::cout << "Received a depth image!" << std::endl;
			sprintf(buffer, "%s/depth%06d.png", impath.c_str(), int(duration.total_milliseconds() - start));
		}
		else if (cv_img_boost_ptr->image.type() == CV_8UC3) {
			std::cout << "Received an RGB image!" << std::endl;
			sprintf(buffer, "%s/rgb%06d.png", impath.c_str(), int(duration.total_milliseconds() - start));
		}
        std::vector<int> compression;
        compression.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression.push_back(0);
		cv::imwrite(buffer, cv_img_boost_ptr->image, compression);
	}

	void synch_callback(const sensor_msgs::Image::ConstPtr& msg)
	{
	    std::cout << "Time lag = " << msg->header.stamp.sec << "." << msg->header.stamp.nsec << std::endl;
	    boost::shared_ptr<sensor_msgs::Image> tracked_object;
		cv_bridge::CvImageConstPtr cv_img_boost_ptr;
		try {
			cv_img_boost_ptr = cv_bridge::toCvShare(*msg, tracked_object);
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
	    boost::posix_time::time_duration duration(time.time_of_day());
	    int delta;
	    char buffer[250];
	    if (cv_img_boost_ptr->image.type() == CV_16UC1) {
            if (gotRGB) {
                std::cout << "Received a depth image!" << std::endl;
                delta = duration.total_milliseconds() - start;
			    sprintf(buffer, "%s/depth%06d.png", impath.c_str(), delta);
			    std::vector<int> compression;
                compression.push_back(CV_IMWRITE_PNG_COMPRESSION);
                compression.push_back(0);
		        cv::imwrite(buffer, cv_img_boost_ptr->image, compression);
		        sprintf(buffer, "%s/rgb%06d.png", impath.c_str(), delta);
		        cv::imwrite(buffer, lastRGB, compression);
		        gotRGB = false;
		        gotDepth = false;
            }
            else {
                std::cout << "Now i set depth!" << std::endl;
                lastDepth = cv_img_boost_ptr->image.clone();
                gotDepth = true;
            }
		}
		else if (cv_img_boost_ptr->image.type() == CV_8UC3) {
		    if (gotRGB) {
			    std::cout << "Received an RGB image!" << std::endl;
			    delta = duration.total_milliseconds() - start;
			    sprintf(buffer, "%s/rgb%06d.png", impath.c_str(), delta);
			    std::vector<int> compression;
                compression.push_back(CV_IMWRITE_PNG_COMPRESSION);
                compression.push_back(0);
		        cv::imwrite(buffer, cv_img_boost_ptr->image, compression);
		        sprintf(buffer, "%s/depth%06d.png", impath.c_str(), delta);
		        cv::imwrite(buffer, lastDepth, compression);
		        gotRGB = false;
		        gotDepth = false;
            }
            else {
                std::cout << "Now i set rgb!" << std::endl;
                lastRGB = cv_img_boost_ptr->image.clone();
                gotRGB = true;
            }

		}
        
	}

}
