/*
 *    decoder.hpp
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

#ifndef LIBAV_IMAGE_TRANSPORT__DECODER_HPP_
#define LIBAV_IMAGE_TRANSPORT__DECODER_HPP_

#include "sensor_msgs/Image.h"
#include "libav_image_transport/Packet.h"

#include "libav_image_transport/libav.hpp"

namespace libav_image_transport
{

class Decoder: public LibAV
{
public:
	Decoder(void);

	void decode(const Packet::ConstPtr &packet, sensor_msgs::ImagePtr& image,
			int &got_image);

	void reconfigure(const int out_width, const int out_height,
			const int out_pix_fmt);

private:
	void init_decoder(const int in_width, const int in_height,
			const int in_pix_fmt, const int codec_ID);

	boost::shared_ptr<Frame> frame_in_;
	boost::shared_ptr<Frame> frame_out_;

	bool has_keyframe_;
	uint32_t previous_packet_;

	int width_out_;
	int height_out_;
	int pix_fmt_out_;
};

} /* namespace libav_image_transport */

#endif /* LIBAV_IMAGE_TRANSPORT__DECODER_HPP_ */
