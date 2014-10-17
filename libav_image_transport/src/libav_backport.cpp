/*
 *    libav_backport.cpp
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


#define __STDC_CONSTANT_MACROS
#include <stdint.h>

extern "C"
{
#include <libavutil/mem.h>
#include <libavcodec/avcodec.h>
}

#include "libav_image_transport/libav_backport.hpp"

void avcodec_free_frame(AVFrame **frame)
{
	AVFrame *f;

	if (!frame || !*frame)
		return;

	f = *frame;

	if (f->extended_data != f->data)
		av_freep(&f->extended_data);

	av_freep(frame);
}
