/*
 *    libav_backport.hpp
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

#ifndef LIBAV_IMAGE_TRANSPORT__LIBAV_BACKPORT_HPP_
#define LIBAV_IMAGE_TRANSPORT__LIBAV_BACKPORT_HPP_

class AVFrame;

#define AVPixelFormat PixelFormat
#define AVCodecID CodecID

#define AV_PIX_FMT_YUV420P PIX_FMT_YUV420P
#define AV_PIX_FMT_YUV422P PIX_FMT_YUV422P
#define AV_PIX_FMT_YUYV422 PIX_FMT_YUYV422
#define AV_PIX_FMT_RGB24 PIX_FMT_RGB24
#define AV_PIX_FMT_BGR24 PIX_FMT_BGR24
#define AV_PIX_FMT_RGBA PIX_FMT_RGBA
#define AV_PIX_FMT_BGRA PIX_FMT_BGRA
#define AV_PIX_FMT_GRAY8 PIX_FMT_GRAY8
#define AV_PIX_FMT_GRAY16LE PIX_FMT_GRAY16LE
#define AV_PIX_FMT_GRAY16BE PIX_FMT_GRAY16BE
#define AV_PIX_FMT_RGB48LE PIX_FMT_RGB48LE
#define AV_PIX_FMT_RGB48BE PIX_FMT_RGB48BE
#define AV_PIX_FMT_BGR48LE PIX_FMT_BGR48LE
#define AV_PIX_FMT_BGR48BE PIX_FMT_BGR48BE

#define AV_CODEC_ID_H264 CODEC_ID_H264
#define AV_CODEC_ID_FFV1 CODEC_ID_FFV1

void avcodec_free_frame(AVFrame **frame);

#endif /* LIBAV_IMAGE_TRANSPORT__LIBAV_BACKPORT_HPP_ */
