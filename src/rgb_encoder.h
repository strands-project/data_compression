#ifndef RGB_ENCODER_H
#define RGB_ENCODER_H

#include "video_encoder.h"

class RGBEncoder : public VideoEncoder {
public:
	RGBEncoder(const std::string& fname);
	~RGBEncoder();
private:
};

#endif