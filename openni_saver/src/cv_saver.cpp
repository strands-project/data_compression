#include "cv_saver.h"

#include <opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <iostream>

namespace cv_saver {

	void init_time()
	{
		boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration duration(time.time_of_day());
		start = duration.total_milliseconds();
	}

	void image_callback(const sensor_msgs::Image::ConstPtr& msg)
	{
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
	    char buffer[50];
	    if (cv_img_boost_ptr->image.type() == CV_16UC1) {
	    	std::cout << "Received a depth image!" << std::endl;
			sprintf(buffer, "images/depth_%06d.png", int(duration.total_milliseconds() - start));
		}
		else if (cv_img_boost_ptr->image.type() == CV_8UC3) {
			std::cout << "Received an RGB image!" << std::endl;
			sprintf(buffer, "images/rgb_%06d.png", int(duration.total_milliseconds() - start));
		}
		//std::cout << cv_img_boost_ptr->image.type() << std::endl;
		//std::cout << CV_32FC1 << std::endl;
		cv::imwrite(buffer, cv_img_boost_ptr->image);
	}

}