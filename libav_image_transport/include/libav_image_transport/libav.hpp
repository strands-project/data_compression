/*
 *    libav.hpp
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

#ifndef LIBAV_IMAGE_TRANSPORT__LIBAV_HPP_
#define LIBAV_IMAGE_TRANSPORT__LIBAV_HPP_

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

struct SwsContext;
class AVCodecContext;
class AVCodec;
class AVDictionary;
class AVFrame;

namespace libav_image_transport
{

class Frame
{
public:
	Frame(const int width, const int height, const int format);
	~Frame(void);

	AVFrame* get_frame(void);

private:
	AVFrame* frame_;
	uint8_t* buf_;
};

class LibAV
{
public:
	~LibAV(void);

protected:
	LibAV(void);

	AVCodec* find_decoder(int codec_ID);
	AVCodec* find_encoder(int codec_ID);

	int codec_open(AVCodecContext *avctx, AVCodec *codec,
			AVDictionary **options);

	void free_context(void);

	AVCodecContext *codec_context_;
	struct SwsContext *sws_context_;

private:
	void init_libav(void);

	static boost::mutex MUTEX;
	static bool INIT_DONE;
};

} /* namespace libav_image_transport */

#endif /* LIBAV_IMAGE_TRANSPORT__LIBAV_HPP_ */
