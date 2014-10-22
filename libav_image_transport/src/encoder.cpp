/*
 *    encoder.cpp
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

#include "libav_image_transport/encoder.hpp"

#include <string.h>
#include <boost/make_shared.hpp>

extern "C"
{
#include <libavutil/mem.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

#include "libav_image_transport/pix_fmt.hpp"

#ifdef BACKPORT_LIBAV
#include "libav_image_transport/libav_backport.hpp"
#endif

namespace libav_image_transport
{

Encoder::Encoder(void) :
		width_out_(-1), height_out_(-1), pix_fmt_out_(-1), codec_ID_(-1), packet_number_(
				0)
{
}

void Encoder::reconfigure(const int out_width, const int out_height,
		const int out_pix_fmt, const int codec_ID, const Config &config)
{
	// Delete old context,
	// i.e. force the encoder to create a new context with new configuration
	free_context();

	width_out_ = out_width;
	height_out_ = out_height;
	pix_fmt_out_ = out_pix_fmt;
	codec_ID_ = codec_ID;
	config_ = config;
}

void Encoder::init_encoder(const int out_width, const int out_height)
{
	/* Declarations */
	AVDictionary *opts;
	AVCodec *codec;
	Config::const_iterator it;

	/* Find the encoder */
	codec = find_encoder(codec_ID_);
	if (!codec)
		throw std::runtime_error("Could not find codec.");

	/* Allocate codec context */
	codec_context_ = avcodec_alloc_context3(codec);
	if (!codec_context_)
		throw std::runtime_error("Could not allocate codec context.");

	/* Set codec configuration */
	codec_context_->pix_fmt = (enum AVPixelFormat) pix_fmt_out_;
	codec_context_->width = out_width;
	codec_context_->height = out_height;
	codec_context_->time_base.den = 1;
	codec_context_->time_base.num = 1000000000;

	opts = NULL;

	if (!config_.empty())
		for (it = config_.begin(); it != config_.end(); ++it)
			av_dict_set(&opts, it->first, it->second, 0);

	/* Open the codec context */
	if (codec_open(codec_context_, codec, &opts) < 0)
		throw std::runtime_error("Could not open codec context.");

	/* Allocate Encoder AVFrame */
	frame_out_ = boost::make_shared<Frame>(out_width, out_height, pix_fmt_out_);

	packet_number_ = 0;
}

void Encoder::encode(const sensor_msgs::Image& image, Packet &packet,
		int &got_packet)
{
	/* Declarations */
	int pix_fmt;
	AVFrame* frame_in;
	AVFrame* frame_out;

#ifdef BACKPORT_LIBAV
	int packet_size;
#else
	AVPacket pkt;
#endif

	const int out_width =
			width_out_ == -1 ? static_cast<int>(image.width) : width_out_;
	const int out_height =
			height_out_ == -1 ? static_cast<int>(image.height) : height_out_;

	/* Get the pixel format of the image */
	if (!pix_fmt_ros2libav(image.encoding, image.is_bigendian, pix_fmt))
		throw std::runtime_error(
				"Image encoding (" + image.encoding + ") can not be handled.");

	packet.pix_fmt = pix_fmt;
	packet.width = image.width;
	packet.height = image.height;

	/* Check if the codec context has to be reinitialized */
	if (!codec_context_ || out_width != codec_context_->width
			|| out_height != codec_context_->height)
	{
		free_context();
		init_encoder(out_width, out_height);
	}

	/* Get local references to the AVFrame structs */
	frame_out = frame_out_->get_frame();

	if (out_width == static_cast<int>(image.width)
			&& out_height == static_cast<int>(image.height)
			&& pix_fmt_out_ == pix_fmt)
		frame_in = frame_out_->get_frame();
	else
	{
		/* Check if the input frame has to be reinitialized */
		frame_in = frame_in_ ? frame_in_->get_frame() : NULL;

		if (!frame_in_ || frame_in->width != static_cast<int>(image.width)
				|| frame_in->height != static_cast<int>(image.height)
				|| frame_in->format != pix_fmt)
		{
			frame_in_ = boost::make_shared<Frame>(image.width, image.height,
					pix_fmt);
			frame_in = frame_in_->get_frame();
		}
	}

	/* Check if the image buffer length matches the expected values */
	if (frame_in->linesize[0] * frame_in->height
			!= static_cast<int>(image.step * image.height))
		throw std::runtime_error("Image buffer size does not match.");

	/* Copy image into AV compatible buffer */
	memcpy(frame_in->data[0], &image.data[0], image.step * image.height);

	/* If the input frame is not the same as the output frame transform it */
	if (frame_in != frame_out)
	{
		/* Get SWS Context */
		sws_context_ = sws_getCachedContext(sws_context_, frame_in->width,
				frame_in->height, (enum AVPixelFormat) frame_in->format,
				frame_out->width, frame_out->height,
				(enum AVPixelFormat) frame_out->format, SWS_BICUBIC, NULL, NULL,
				NULL);
		if (!sws_context_)
			throw std::runtime_error("Could not initialize sws context.");

		/* Transform Image */
		sws_scale(sws_context_, frame_in->data, frame_in->linesize, 0,
				frame_in->height, frame_out->data, frame_out->linesize);
	}

	/* Store the PTS in the AVFrame */
	frame_out->pts = (static_cast<int64_t>(image.header.stamp.sec) << 32)
			| image.header.stamp.nsec;

#ifdef BACKPORT_LIBAV
	/* Encode Image */
	packet_size = avcodec_encode_video(codec_context_, buf_, BUF_SIZE_,
			frame_out);

	if (packet_size < 0)
		throw std::runtime_error("Could not encode image.");
	else if (packet_size == 0)
		got_packet = 0;
	else
		got_packet = 1;
#else
	/* Initialize the packet */
	av_init_packet(&pkt);
	pkt.data = NULL;
	pkt.size = 0;

	/* Encode Image */
	if (avcodec_encode_video2(codec_context_, &pkt, frame_out, &got_packet) < 0)
		throw std::runtime_error("Could not encode image.");
#endif

	if (!got_packet)
		return;

	/* Fill the packet message */
	packet.codec_ID = codec_context_->codec_id;
	packet.seq = ++packet_number_;
	packet.compressed_pix_fmt = pix_fmt_out_;
	packet.compressed_width = codec_context_->width;
	packet.compressed_height = codec_context_->height;
	packet.header = image.header;

#ifdef BACKPORT_LIBAV
	packet.pts = codec_context_->coded_frame->pts;
	packet.keyframe = static_cast<bool>(codec_context_->coded_frame->key_frame);
	packet.data.resize(packet_size);
	packet.data.assign(buf_, buf_ + packet_size);
#else
	packet.pts = pkt.pts;
	packet.keyframe = pkt.flags & AV_PKT_FLAG_KEY ? true : false;
	packet.data.resize(pkt.size);
	packet.data.assign(pkt.data, pkt.data + pkt.size);

	/* Free the packet data */
	if (pkt.destruct)
		pkt.destruct(&pkt);
	else
		av_free_packet(&pkt);
#endif
}

} /* namespace libav_image_transport */
