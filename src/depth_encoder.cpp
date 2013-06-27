#include "depth_encoder.h"

DepthEncoder::DepthEncoder(const std::string& fname) :
	VideoEncoder(fname, PIX_FMT_YUV420P,
		PIX_FMT_GRAY16, AV_CODEC_ID_MPEG1VIDEO,
	    32, 4000000, 2)
{

}

DepthEncoder::~DepthEncoder()
{
	
}