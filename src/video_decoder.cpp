#include "video_decoder.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <cstdio>

VideoDecoder::VideoDecoder(const std::string& newfname,
    enum PixelFormat newsrc_pix_fmt,
    enum PixelFormat newdest_pix_fmt,
    enum AVCodecID src_codec,
    int dest_pix_bytes) : fname(newfname),
        src_pix_fmt(newsrc_pix_fmt), dest_pix_fmt(newdest_pix_fmt)
{
    dest_linesize[0] = dest_pix_bytes*640;
    finished = false;
    first_iter = true;
	avcodec_register_all();

    c = NULL;
    
    av_init_packet(&avpkt);

    /* set end of buffer to 0 (this ensures that no overreading happens for damaged mpeg streams) */
    memset(inbuf + INBUF_SIZE, 0, FF_INPUT_BUFFER_PADDING_SIZE);

    printf("Video decoding\n");

    /* find the mpeg1 video decoder */
    codec = avcodec_find_decoder(src_codec);
    if (!codec) {
        fprintf(stderr, "codec not found\n");
        exit(1);
    }

    c = avcodec_alloc_context3(codec);
    picture = avcodec_alloc_frame();

    if(codec->capabilities&CODEC_CAP_TRUNCATED)
        c->flags|= CODEC_FLAG_TRUNCATED; /* we do not send complete frames */

    /* For some codecs, such as msmpeg4 and mpeg4, width and height
       MUST be initialized there because this information is not
       available in the bitstream. */

    /* open it */
    if (avcodec_open2(c, codec, NULL) < 0) {
        fprintf(stderr, "could not open codec\n");
        exit(1);
    }

    /* the codec gives us the frame size, in samples */

    f = fopen(fname.c_str(), "rb");
    if (!f) {
        fprintf(stderr, "could not open %s\n", fname.c_str());
        exit(1);
    }

    frame = 0;
}

VideoDecoder::~VideoDecoder()
{
    fclose(f);

    avcodec_close(c);
    av_free(c);
    av_free(picture);
    printf("\n");
}

bool VideoDecoder::getFrame(cv::Mat& cvMat)
{
    bool rtn;
    if (finished) {
        return false;
    }
    do {
        if (first_iter) {
            avpkt.size = fread(inbuf, 1, INBUF_SIZE, f);
            avpkt.data = inbuf;
            first_iter = false;
        }
        if (avpkt.size > 0) {
            len = avcodec_decode_video2(c, picture, &got_picture, &avpkt);
            if (len < 0) {
                fprintf(stderr, "Error while decoding frame %d\n", frame);
                exit(1);
            }
            if (got_picture) {
                printf("saving frame %3d\n", frame);
                fflush(stdout);

                AVFrameToCVMat(picture, cvMat, src_pix_fmt);
                /* the picture is allocated by the decoder. no need to
                   free it */
                frame++;
            }
            avpkt.size -= len;
            avpkt.data += len;
            if (avpkt.size <= 0) {
                first_iter = true;
            }
        }
        else {
            finished = true;
            break;
        }
    }
    while (!got_picture);
    if (!finished) {
        return true;
    }
    /* some codecs, such as MPEG, transmit the I and P frame with a
   	latency of one frame. You must do the following to have a
   	chance to get the last frame of the video */
    avpkt.data = NULL;
    avpkt.size = 0;
    len = avcodec_decode_video2(c, picture, &got_picture, &avpkt);
    if (!got_picture) {
        return false;
    }
    printf("saving last frame %3d\n", frame);
    fflush(stdout);

    AVFrameToCVMat(picture, cvMat, src_pix_fmt);
    /* the picture is allocated by the decoder. no need to
       free it */
    frame++;
    return true;

}

void VideoDecoder::AVFrameToCVMat(AVFrame* avFrame, cv::Mat& cvMat, enum PixelFormat pix_fmt)
{
    struct SwsContext* img_convert_ctx = 0;
    cv::Size s = cvMat.size();
    //int linesize[4] = {3 * s.width, 0, 0, 0};

    img_convert_ctx = sws_getContext(s.width, s.height,
        pix_fmt, s.width, s.height, dest_pix_fmt, SWS_BILINEAR, 0, 0, 0);

    if (img_convert_ctx == 0) {
        return;
    }

    sws_scale(img_convert_ctx, avFrame->data, avFrame->linesize, 0, s.height,
        (uint8_t **) & (cvMat.data), dest_linesize);
    sws_freeContext(img_convert_ctx);
}