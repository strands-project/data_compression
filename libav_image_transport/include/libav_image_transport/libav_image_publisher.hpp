/*
 *    libav_image_publisher.hpp
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

#ifndef LIBAV_IMAGE_TRANSPORT__LIBAV_IMAGE_PUBLISHER_HPP_
#define LIBAV_IMAGE_TRANSPORT__LIBAV_IMAGE_PUBLISHER_HPP_

#include "dynamic_reconfigure/server.h"
#include "image_transport/simple_publisher_plugin.h"

#include "libav_image_transport/Packet.h"
#include "libav_image_transport/libavPublisherConfig.h"

#include "libav_image_transport/worker.hpp"
#include "libav_image_transport/encoder.hpp"

namespace libav_image_transport
{

class LibAVImagePublisher: public image_transport::SimplePublisherPlugin<
		libav_image_transport::Packet>
{
public:
	LibAVImagePublisher(void)
	{
		worker_ = boost::make_shared<Worker>();
		worker_->resize(30);
		worker_->start();
		encoder_ = boost::make_shared<Encoder>();
	}

	virtual ~LibAVImagePublisher(void)
	{
		worker_->stop();
	}

	virtual std::string getTransportName() const
	{
		return "libav";
	}

protected:
	// Overridden to set up reconfigure server
	virtual void advertiseImpl(ros::NodeHandle &nh,
			const std::string &base_topic, uint32_t queue_size,
			const image_transport::SubscriberStatusCallback &user_connect_cb,
			const image_transport::SubscriberStatusCallback &user_disconnect_cb,
			const ros::VoidPtr &tracked_object, bool latch);

	virtual void publish(const sensor_msgs::Image& message,
			const PublishFn& publish_fn) const;
	virtual void encode(const sensor_msgs::Image& image,
			const PublishFn& publish_fn) const;

	typedef libav_image_transport::libavPublisherConfig Config;
	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

	void reconfigure(Config& config, uint32_t level);

	boost::shared_ptr<Worker> worker_;
	boost::shared_ptr<ReconfigureServer> reconfigure_server_;
	boost::shared_ptr<Encoder> encoder_;
};

} /* namespace libav_image_transport */

#endif /* LIBAV_IMAGE_TRANSPORT__LIBAV_IMAGE_PUBLISHER_HPP_ */
