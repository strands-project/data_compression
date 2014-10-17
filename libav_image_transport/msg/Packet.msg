# Description of the uncompressed image
int16   pix_fmt
uint16  width
uint16  height
Header 	header

# Description of the compressed image
bool    keyframe
int16   compressed_pix_fmt
uint16  compressed_width
uint16  compressed_height

# Codec information
int16   codec_ID
uint32  seq
int64   pts

# Compressed data
uint8[] data
