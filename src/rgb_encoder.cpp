#include "rgb_encoder.h"

RGBEncoder::RGBEncoder(const std::string& fname) :
	VideoEncoder(fname, PIX_FMT_YUV420P,
	    PIX_FMT_BGR24, AV_CODEC_ID_MPEG1VIDEO,
	    32, 4000000, 3)
{

}

RGBEncoder::~RGBEncoder()
{

}