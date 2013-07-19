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

#include "libav_image_transport/config.hpp"
#include "libav_image_transport/pix_fmt.hpp"


namespace libav_image_transport
{

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

} /* namespace libav_image_transport */
