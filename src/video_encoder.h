#ifndef VIDEO_ENCODER_H
#define VIDEO_ENCODER_H

#define __STDC_CONSTANT_MACROS

#ifdef HAVE_AV_CONFIG_H
#undef HAVE_AV_CONFIG_H
#endif

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/channel_layout.h>
#include <libavutil/common.h>
#include <libavutil/imgutils.h>
#include <libavutil/mathematics.h>
#include <libavutil/samplefmt.h>
#include <libswscale/swscale.h>
}

#include <opencv.hpp>

class VideoEncoder {
private:
	std::string fname;
	enum PixelFormat src_pix_fmt;
	AVCodec* codec;
	AVCodecContext* c;
	int i, ret, x, y, got_output;
	FILE* f;
	AVFrame* picture;
	AVPacket pkt;
	uint8_t endcode[4];
	int src_linesize[4];
public:
	void addFrame(const cv::Mat& cvMat);
	VideoEncoder(const std::string& newfname,
	    enum PixelFormat dest_pix_fmt,
	    enum PixelFormat newsrc_pix_fmt,
	    enum AVCodecID dest_codec,
	    int dest_pix_bits,
	    int dest_bit_rate,
    	int src_pix_bytes);
	~VideoEncoder();
private:
	void CVMatToAVFrame(const cv::Mat& cvMat, AVFrame* avFrame, enum PixelFormat pix_fmt);
};
#endif