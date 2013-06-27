#ifndef DEPTH_DECODER_H
#define DEPTH_DECODER_H

#include "video_decoder.h"

class DepthDecoder : public VideoDecoder {
public:
	DepthDecoder(const std::string& fname);
	~DepthDecoder();
private:

};
#endif