/*
 *    pix_fmt.hpp
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

#ifndef LIBAV_IMAGE_TRANSPORT__PIX_FMT_HPP_
#define LIBAV_IMAGE_TRANSPORT__PIX_FMT_HPP_

#include <string>

namespace libav_image_transport
{

bool pix_fmt_libav2ros(const int pix_fmt, std::string &encoding,
		unsigned char &is_bigendian);

bool pix_fmt_ros2libav(const std::string &encoding,
		const unsigned char is_bigendian, int &pix_fmt);

} /* namespace libav_image_transport */

#endif /* LIBAV_IMAGE_TRANSPORT__PIX_FMT_HPP_ */
