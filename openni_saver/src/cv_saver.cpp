#include "cv_saver.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <iostream>

namespace cv_saver {

	void init_saver(const std::string& path)
	{
        impath = path;
        depths.clear();
        rgbs.clear();
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
	    int sec = msg->header.stamp.sec;
	    int nsec = msg->header.stamp.nsec;
	    std::cout << "Time lag = " << sec << "." << nsec << std::endl;
	    std::cout << "Depths: " << depths.size() << " , RGBs: " << rgbs.size() << std::endl;
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
            if (!rgbs.empty()) {
                std::cout << "Received a depth image!" << std::endl;
                delta = duration.total_milliseconds() - start;
			    sprintf(buffer, "%s/depth%06d-%010d-%010d.png", impath.c_str(), delta, sec, nsec);
			    std::vector<int> compression;
                compression.push_back(CV_IMWRITE_PNG_COMPRESSION);
                compression.push_back(0);
		        cv::imwrite(buffer, cv_img_boost_ptr->image, compression);
		        sprintf(buffer, "%s/rgb%06d-%010d-%010d.png", impath.c_str(), delta, rgb_secs.front(), rgb_nsecs.front());
		        cv::imwrite(buffer, rgbs.front(), compression);
		        rgbs.pop_front();
		        rgb_secs.pop_front();
		        rgb_nsecs.pop_front();
            }
            else {
                std::cout << "Now i set depth!" << std::endl;
                depths.push_back(cv_img_boost_ptr->image.clone());
                depth_secs.push_back(sec);
                depth_nsecs.push_back(nsec);
            }
		}
		else if (cv_img_boost_ptr->image.type() == CV_8UC3) {
		    if (!depths.empty()) {
			    std::cout << "Received an RGB image!" << std::endl;
			    delta = duration.total_milliseconds() - start;
			    sprintf(buffer, "%s/rgb%06d-%010d-%010d.png", impath.c_str(), delta, sec, nsec);
			    std::vector<int> compression;
                compression.push_back(CV_IMWRITE_PNG_COMPRESSION);
                compression.push_back(0);
		        cv::imwrite(buffer, cv_img_boost_ptr->image, compression);
		        sprintf(buffer, "%s/depth%06d-%010d-%010d.png", impath.c_str(), delta, depth_secs.front(), depth_nsecs.front());
		        cv::imwrite(buffer, depths.front(), compression);
		        depths.pop_front();
		        depth_secs.pop_front();
		        depth_nsecs.pop_front();
            }
            else {
                std::cout << "Now i set rgb!" << std::endl;
                rgbs.push_back(cv_img_boost_ptr->image.clone());
                rgb_secs.push_back(sec);
                rgb_nsecs.push_back(nsec);
            }

		}
        
	}

}
