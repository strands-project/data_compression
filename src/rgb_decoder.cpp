#include "rgb_decoder.h"

RGBDecoder::RGBDecoder(const std::string& fname) :
	VideoDecoder(fname, PIX_FMT_YUV420P,
	    PIX_FMT_BGR24, AV_CODEC_ID_MPEG1VIDEO, 3)
{

}

RGBDecoder::~RGBDecoder()
{
	
}