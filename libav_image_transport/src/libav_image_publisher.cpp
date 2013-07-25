/*
 *    libav_image_subscriber.cpp
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

#include "libav_image_transport/libav_image_publisher.hpp"

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

void LibAVImagePublisher::advertiseImpl(ros::NodeHandle &nh,
		const std::string &base_topic, uint32_t queue_size,
		const image_transport::SubscriberStatusCallback &user_connect_cb,
		const image_transport::SubscriberStatusCallback &user_disconnect_cb,
		const ros::VoidPtr &tracked_object, bool latch)
{
	// super call
	image_transport::SimplePublisherPlugin<libav_image_transport::Packet>::advertiseImpl(
			nh, base_topic, queue_size, user_connect_cb, user_disconnect_cb,
			tracked_object, latch);

	// setup reconfigure server
	reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
	reconfigure_server_->setCallback(
			static_cast<ReconfigureServer::CallbackType>(boost::bind(
					&LibAVImagePublisher::reconfigure, this, _1, _2)));
}

void LibAVImagePublisher::publish(const sensor_msgs::Image& message,
		const PublishFn& publish_fn) const
{
	worker_->schedule(
			boost::function<void()>(
					boost::bind(&LibAVImagePublisher::encode, this, message,
							publish_fn)));
}

void LibAVImagePublisher::encode(const sensor_msgs::Image& image,
		const PublishFn& publish_fn) const
{
	int got_packet;

	libav_image_transport::Packet packet;
	encoder_->encode(image, packet, got_packet);

	if (got_packet)
		publish_fn(packet);
}

void LibAVImagePublisher::reconfigure(Config& config, uint32_t level)
{
	int pix_fmt;
	int codec_ID;
	Encoder::Config codec_config;

	if (config.codec == "h264")
	{
		codec_ID = AV_CODEC_ID_H264;

		char gop_size[10];
		snprintf(gop_size, 8, "%i", config.gop_size);

		codec_config.push_back(
				Encoder::Option("preset", config.x264_preset.c_str()));
		codec_config.push_back(Encoder::Option("gop_size", gop_size));
	}
	else if (config.codec == "ffv1")
		codec_ID = AV_CODEC_ID_FFV1;
	else
		throw std::runtime_error(
				"Invalid codec (" + config.codec + ") encountered");

	if (config.pixel_format == "rgb")
		pix_fmt = AV_PIX_FMT_RGB24;
	else if (config.pixel_format == "yuv")
		pix_fmt = AV_PIX_FMT_YUV420P;
	else if (config.pixel_format == "gray8")
		pix_fmt = AV_PIX_FMT_GRAY8;
	else if (config.pixel_format == "gray16")
		pix_fmt = AV_PIX_FMT_GRAY16LE;
	else
		throw std::runtime_error(
				"Invalid pixel format (" + config.pixel_format
						+ ") encountered.");

	encoder_->reconfigure(config.encoded_width, config.encoded_height, pix_fmt,
			codec_ID, codec_config);
}

} /* namespace libav_image_transport */
