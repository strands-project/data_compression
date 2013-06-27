#include "depth_decoder.h"

DepthDecoder::DepthDecoder(const std::string& fname) :
	VideoDecoder(fname, PIX_FMT_YUV420P, 
		PIX_FMT_GRAY16, AV_CODEC_ID_MPEG1VIDEO, 2)
{
	
}

DepthDecoder::~DepthDecoder()
{
	
}