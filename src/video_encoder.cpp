#include "video_encoder.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <cstdio>

#define INBUF_SIZE 4096
#define AUDIO_INBUF_SIZE 20480
#define AUDIO_REFILL_THRESH 4096

VideoEncoder::VideoEncoder(const std::string& newfname,
    enum PixelFormat dest_pix_fmt,
    enum PixelFormat newsrc_pix_fmt,
    enum AVCodecID dest_codec,
    int dest_pix_bits,
    int dest_bit_rate,
    int src_pix_bytes) : fname(newfname), src_pix_fmt(newsrc_pix_fmt)
{
    avcodec_register_all();
    c = NULL;
    endcode[2] = 1;
    endcode[3] = 0xb7;
    src_linesize[0] = src_pix_bytes*640;

    printf("Video encoding\n");

    /* find the mpeg1 video encoder */
    codec = avcodec_find_encoder(dest_codec); // PARAMETER
    if (!codec) {
        fprintf(stderr, "codec not found\n");
        exit(1);
    }

    c = avcodec_alloc_context3(codec);
    picture = avcodec_alloc_frame();

    /* put sample parameters */
    c->bit_rate = dest_bit_rate;//4000000;
    /* resolution must be a multiple of two */
    c->width = 640;
    c->height = 480;
    /* frames per second */
    c->time_base = (AVRational){1, 30};
    c->gop_size = 10; /* emit one intra frame every ten frames */
    c->max_b_frames = 1;
    c->pix_fmt = dest_pix_fmt;//PIX_FMT_YUV420P; // PARAMETER

    /* open it */
    if (avcodec_open2(c, codec, NULL) < 0) {
        fprintf(stderr, "could not open codec\n");
        exit(1);
    }

    f = fopen(fname.c_str(), "wb");
    if (!f) {
        fprintf(stderr, "could not open %s\n", fname.c_str());
        exit(1);
    }

    ret = av_image_alloc(picture->data, picture->linesize, c->width, c->height,
                         c->pix_fmt, dest_pix_bits); // 32 PARAMETER
    if (ret < 0) {
        fprintf(stderr, "could not alloc raw picture buffer\n");
        exit(1);
    }
    picture->format = c->pix_fmt;
    picture->width  = c->width;
    picture->height = c->height;

    i = 0;
}

VideoEncoder::~VideoEncoder()
{
    for(got_output = 1; got_output; i++) {
        fflush(stdout);

        ret = avcodec_encode_video2(c, &pkt, NULL, &got_output);
        if (ret < 0) {
            fprintf(stderr, "error encoding frame\n");
            exit(1);
        }

        if (got_output) {
            printf("encoding frame %3d (size=%5d)\n", i, pkt.size);
            fwrite(pkt.data, 1, pkt.size, f);
            av_free_packet(&pkt);
        }
    }

    std::cout << "Constructor was called!" << std::endl;

    /* add sequence end code to have a real mpeg file */
    fwrite(endcode, 1, sizeof(endcode), f);
    fclose(f);

    avcodec_close(c);
    av_free(c);
    av_freep(&picture->data[0]);
    avcodec_free_frame(&picture);
    printf("\n");
}

void VideoEncoder::addFrame(const cv::Mat& cvMat)
{
    av_init_packet(&pkt);
    pkt.data = NULL; // packet data will be allocated by the encoder
    pkt.size = 0;
    fflush(stdout);

    CVMatToAVFrame(cvMat, picture, c->pix_fmt);
    picture->pts = i;
    ret = avcodec_encode_video2(c, &pkt, picture, &got_output);

    if (ret < 0) {
        fprintf(stderr, "error encoding frame\n");
        exit(1);
    }

    if (got_output) {
        printf("encoding frame %3d (size=%5d)\n", i, pkt.size);
        fwrite(pkt.data, 1, pkt.size, f);
        av_free_packet(&pkt);
    }
    i++;
}

void VideoEncoder::CVMatToAVFrame(const cv::Mat& cvMat, AVFrame* avFrame, enum PixelFormat pix_fmt)
{
    struct SwsContext* img_convert_ctx = 0;
    cv::Size s = cvMat.size();
    //int linesize[4] = {3 * s.width, 0, 0, 0};

    img_convert_ctx = sws_getContext(s.width, s.height,
        src_pix_fmt, s.width, s.height, pix_fmt, SWS_BILINEAR, 0, 0, 0);

    if (img_convert_ctx == 0) {
        return;
    }

    sws_scale(img_convert_ctx, &cvMat.data, src_linesize, 0, s.height,
        avFrame->data, avFrame->linesize);
    sws_freeContext(img_convert_ctx);
    /*cv::Size s = cvMat.size();
    avpicture_fill(avFrame, cvMat.data,
                       src_pix_fmt, s.width, s.height);*/
}