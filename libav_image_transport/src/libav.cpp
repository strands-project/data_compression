/*
 *    libav.cpp
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

#include "libav_image_transport/libav.hpp"

#include <stdexcept>

extern "C"
{
#include <libavutil/mem.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

#include "libav_image_transport/config.hpp"

#ifdef BACKPORT_LIBAV
#include "libav_image_transport/libav_backport.hpp"
#endif

namespace libav_image_transport
{

Frame::Frame(const int width, const int height, const int format) :
		frame_(NULL), buf_(NULL)
{
	int num_bytes;

	frame_ = avcodec_alloc_frame();
	if (!frame_)
		throw std::runtime_error("Could not allocate frame.");

	frame_->format = format;
	frame_->width = width;
	frame_->height = height;

	num_bytes = avpicture_get_size((enum AVPixelFormat) format, width, height);
	if (num_bytes < 0)
		throw std::runtime_error("Could not calculate buffer size for frame.");

	buf_ = (uint8_t*) av_malloc(num_bytes * sizeof(uint8_t));
	if (!buf_)
		throw std::runtime_error("Could not allocate buffer for frame.");

	if (num_bytes
			!= avpicture_fill((AVPicture*) frame_, buf_,
					(enum AVPixelFormat) format, width, height))
		throw std::runtime_error("Buffer size for frame does not match.");
}

Frame::~Frame(void)
{
	av_freep(&buf_);
	avcodec_free_frame(&frame_);
}

AVFrame* Frame::get_frame(void)
{
	return frame_;
}

boost::mutex LibAV::MUTEX;
bool LibAV::INIT_DONE = false;

LibAV::LibAV(void) :
		codec_context_(NULL), sws_context_(NULL)
{
}

LibAV::~LibAV(void)
{
	free_context();
}

AVCodec* LibAV::find_decoder(int codec_ID)
{
	boost::mutex::scoped_lock lock(MUTEX);

	if (!INIT_DONE)
		init_libav();

	return avcodec_find_decoder((enum AVCodecID) codec_ID);
}

AVCodec* LibAV::find_encoder(int codec_ID)
{
	boost::mutex::scoped_lock lock(MUTEX);

	if (!INIT_DONE)
		init_libav();

	return avcodec_find_encoder((enum AVCodecID) codec_ID);
}

int LibAV::codec_open(AVCodecContext *avctx, AVCodec *codec,
		AVDictionary **options)
{
	boost::mutex::scoped_lock lock(MUTEX);

	if (!INIT_DONE)
		init_libav();

	return avcodec_open2(avctx, codec, options);
}

void LibAV::init_libav(void)
{
	/* Initialize the libav library */
	avcodec_register_all();
	INIT_DONE = true;
}

void LibAV::free_context(void)
{
	boost::mutex::scoped_lock lock(MUTEX);

	if (codec_context_)
	{
		avcodec_close(codec_context_);
		av_freep(&codec_context_);
	}

	if (sws_context_)
	{
		sws_freeContext(sws_context_);
		sws_context_ = NULL;
	}
}

} /* namespace libav_image_transport */
