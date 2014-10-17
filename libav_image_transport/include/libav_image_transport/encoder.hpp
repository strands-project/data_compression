/*
 *    encoder.hpp
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

#ifndef LIBAV_IMAGE_TRANSPORT__ENCODER_HPP_
#define LIBAV_IMAGE_TRANSPORT__ENCODER_HPP_

#include <vector>
#include <utility>

#include "sensor_msgs/Image.h"
#include "libav_image_transport/Packet.h"

#include "libav_image_transport/libav.hpp"
#include "libav_image_transport/config.hpp"

namespace libav_image_transport
{

class Encoder: public LibAV
{
public:
	typedef std::pair<const char*, const char*> Option;
	typedef std::vector<Option> Config;

	Encoder(void);

	void encode(const sensor_msgs::Image& image, Packet &packet,
			int &got_packet);

	void reconfigure(const int out_width, const int out_height,
			const int out_pix_fmt, const int codec_ID, const Config &config);

private:
	void init_encoder(const int in_width, const int in_height);

	boost::shared_ptr<Frame> frame_in_;
	boost::shared_ptr<Frame> frame_out_;

#ifdef BACKPORT_LIBAV
	const static int BUF_SIZE_ = 500000;
	uint8_t buf_[BUF_SIZE_];
#endif

	int width_out_;
	int height_out_;
	int pix_fmt_out_;
	int codec_ID_;
	Config config_;

	uint32_t packet_number_;
};

} /* namespace libav_image_transport */

#endif /* LIBAV_IMAGE_TRANSPORT__ENCODER_HPP_ */
