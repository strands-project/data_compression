#include "depth_decoder.h"

DepthDecoder::DepthDecoder(const std::string& fname) :
	VideoDecoder(fname, PIX_FMT_GRAY16LE, 
		PIX_FMT_GRAY16, AV_CODEC_ID_FFV1, 2)
{
	
}

DepthDecoder::~DepthDecoder()
{
	
}