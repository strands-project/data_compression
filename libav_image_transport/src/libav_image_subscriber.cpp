/*
 *    libav_image_publisher.cpp
 *
 *    Copyright 2013 Dominique Hunziker
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *
 *     \author/s: Dominique Hunziker
 */

#include "libav_image_transport/libav_image_subscriber.hpp"

#include <boost/bind.hpp>

extern "C"
{
#include "libavcodec/avcodec.h"
}

#include "libav_image_transport/config.hpp"

#ifdef BACKPORT_LIBAV
#include "libav_image_transport/libav_backport.hpp"
#endif

namespace libav_image_transport
{

void LibAVImageSubscriber::subscribeImpl(ros::NodeHandle& nh,
		const std::string& base_topic, uint32_t queue_size,
		const Callback& callback, const ros::VoidPtr& tracked_object,
		const image_transport::TransportHints& transport_hints)
{
	// super call
	image_transport::SimpleSubscriberPlugin<libav_image_transport::Packet>::subscribeImpl(
			nh, base_topic, queue_size, callback, tracked_object,
			transport_hints);

	// setup reconfigure server
	reconfigure_server_ = boost::make_shared<ReconfigureServer>(
			ros::NodeHandle(transport_hints.getParameterNH(),
					getTopic().substr(1)));
	reconfigure_server_->setCallback(
			static_cast<ReconfigureServer::CallbackType>(boost::bind(
					&LibAVImageSubscriber::reconfigure, this, _1, _2)));
}

void LibAVImageSubscriber::internalCallback(const Packet::ConstPtr &message,
		const Callback& user_cb)
{
	worker_->schedule(
			boost::function<void()>(
					boost::bind(&LibAVImageSubscriber::decode, this, message,
							user_cb)));
}

void LibAVImageSubscriber::decode(const Packet::ConstPtr &packet,
		const Callback& user_cb)
{
	int got_image;

	sensor_msgs::ImagePtr image = boost::make_shared<sensor_msgs::Image>();
	decoder_->decode(packet, image, got_image);

	if (got_image)
		user_cb(image);
}

void LibAVImageSubscriber::reconfigure(Config& config, uint32_t level)
{
	int pix_fmt;

	if (config.pixel_format == "default")
		pix_fmt = -1;
	else if (config.pixel_format == "rgb")
		pix_fmt = AV_PIX_FMT_RGB24;
	else if (config.pixel_format == "bgr")
		pix_fmt = AV_PIX_FMT_BGR24;
	else if (config.pixel_format == "yuv")
		pix_fmt = AV_PIX_FMT_UYVY422;
	else if (config.pixel_format == "gray8")
		pix_fmt = AV_PIX_FMT_GRAY8;
	else if (config.pixel_format == "gray16")
		pix_fmt = AV_PIX_FMT_GRAY16LE;
	else
		throw std::runtime_error(
				"Invalid pixel format (" + config.pixel_format
						+ ") encountered");

	decoder_->reconfigure(config.width, config.height, pix_fmt);
}

} /* namespace libav_image_transport */
