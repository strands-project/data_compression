#ifndef DEPTH_ENCODER_H
#define DEPTH_ENCODER_H

#include "video_encoder.h"

class DepthEncoder : public VideoEncoder {
public:
	DepthEncoder(const std::string& fname);
	~DepthEncoder();
private:

};
#endif