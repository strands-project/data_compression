/*
 *    pix_fmt.cpp
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

#include "libav_image_transport/pix_fmt.hpp"
#include "libav_image_transport/config.hpp"

#include <libavutil/pixfmt.h>

#include "sensor_msgs/image_encodings.h"

#ifdef BACKPORT_LIBAV
#include "libav_image_transport/libav_backport.hpp"
#endif

namespace libav_image_transport
{

bool pix_fmt_libav2ros(const int pix_fmt, std::string &encoding,
		unsigned char &is_bigendian)
{
	is_bigendian = 0;

	switch (pix_fmt)
	{
	case AV_PIX_FMT_RGB24:
		encoding = sensor_msgs::image_encodings::RGB8;
		break;

	case AV_PIX_FMT_RGBA:
		encoding = sensor_msgs::image_encodings::RGBA8;
		break;

	case AV_PIX_FMT_BGR24:
		encoding = sensor_msgs::image_encodings::BGR8;
		break;

	case AV_PIX_FMT_BGRA:
		encoding = sensor_msgs::image_encodings::BGRA8;
		break;

	case AV_PIX_FMT_GRAY8:
		encoding = sensor_msgs::image_encodings::MONO8;
		break;

	case AV_PIX_FMT_GRAY16BE:
		is_bigendian = 1;

	case AV_PIX_FMT_GRAY16LE:
		encoding = sensor_msgs::image_encodings::MONO16;
		break;

	case AV_PIX_FMT_RGB48BE:
		is_bigendian = 1;

	case AV_PIX_FMT_RGB48LE:
		encoding = sensor_msgs::image_encodings::RGB16;
		break;

	case AV_PIX_FMT_BGR48BE:
		is_bigendian = 1;

	case AV_PIX_FMT_BGR48LE:
		encoding = sensor_msgs::image_encodings::BGR16;
		break;

	case AV_PIX_FMT_UYVY422:
		encoding = sensor_msgs::image_encodings::YUV422;
		break;

	default:
		return false;
	}

	return true;
}

bool pix_fmt_ros2libav(const std::string &encoding,
		const unsigned char is_bigendian, int &pix_fmt)
{
	if (encoding == sensor_msgs::image_encodings::RGB8)
		pix_fmt = AV_PIX_FMT_RGB24;
	else if (encoding == sensor_msgs::image_encodings::RGBA8)
		pix_fmt = AV_PIX_FMT_RGBA;
	else if (encoding == sensor_msgs::image_encodings::BGR8)
		pix_fmt = AV_PIX_FMT_BGR24;
	else if (encoding == sensor_msgs::image_encodings::BGRA8)
		pix_fmt = AV_PIX_FMT_BGRA;
	else if (encoding == sensor_msgs::image_encodings::MONO8
			|| encoding == sensor_msgs::image_encodings::TYPE_8UC1)
		pix_fmt = AV_PIX_FMT_GRAY8;
	else if (encoding == sensor_msgs::image_encodings::MONO16
			|| encoding == sensor_msgs::image_encodings::TYPE_16UC1)
	{
		if (is_bigendian)
			pix_fmt = AV_PIX_FMT_GRAY16BE;
		else
			pix_fmt = AV_PIX_FMT_GRAY16LE;
	}
	else if (encoding == sensor_msgs::image_encodings::RGB16)
	{
		if (is_bigendian)
			pix_fmt = AV_PIX_FMT_RGB48BE;
		else
			pix_fmt = AV_PIX_FMT_RGB48LE;
	}
	else if (encoding == sensor_msgs::image_encodings::BGR16)
	{
		if (is_bigendian)
			pix_fmt = AV_PIX_FMT_BGR48BE;
		else
			pix_fmt = AV_PIX_FMT_BGR48LE;
	}
	else if (encoding == sensor_msgs::image_encodings::YUV422)
		pix_fmt = AV_PIX_FMT_UYVY422;
	else
		return false;

	return true;
}

} /* namespace libav_image_transport */
