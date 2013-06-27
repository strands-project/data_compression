#ifndef RGB_DECODER_H
#define RGB_DECODER_H

#include "video_decoder.h"

class RGBDecoder : public VideoDecoder {
public:
	RGBDecoder(const std::string& fname);
	~RGBDecoder();
private:

};
#endif