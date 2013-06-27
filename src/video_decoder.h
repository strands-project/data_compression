#ifndef VIDEO_DECODER_H
#define VIDEO_DECODER_H

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

#define INBUF_SIZE 4096
#define AUDIO_INBUF_SIZE 20480
#define AUDIO_REFILL_THRESH 4096

class VideoDecoder {
private:
    bool first_iter;
    bool finished;
    std::string fname;
    enum PixelFormat dest_pix_fmt;
    enum PixelFormat src_pix_fmt;
    int dest_linesize[4];
    AVCodec* codec;
    AVCodecContext* c;
    int frame, got_picture, len;
    FILE* f;
    AVFrame* picture;
    uint8_t inbuf[INBUF_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];
    char buf[1024];
    AVPacket avpkt;
public:
    VideoDecoder(const std::string& newfname,
        enum PixelFormat newsrc_pix_fmt,
        enum PixelFormat newdest_pix_fmt,
        enum AVCodecID src_codec,
        int dest_pix_bytes);
    ~VideoDecoder();
    bool getFrame(cv::Mat& cvMat);
private:
    void AVFrameToCVMat(AVFrame* avFrame, cv::Mat& rgbMat, enum PixelFormat pix_fmt);
};
#endif