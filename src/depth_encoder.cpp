#include "depth_encoder.h"

DepthEncoder::DepthEncoder(const std::string& fname) :
	VideoEncoder(fname, PIX_FMT_GRAY16,
		PIX_FMT_GRAY16, AV_CODEC_ID_FFV1,
	    16, 4000000, 2)
{

}

DepthEncoder::~DepthEncoder()
{
	
}