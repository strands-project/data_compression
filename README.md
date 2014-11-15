data_compression
================

Video encoding for OpenNI topics to record them to a rosbag and replay through reconstruction of point clouds.

Includes libav_image_transport by Dominique Hunziker: https://github.com/rapyuta/libav_image_transport.

## Compression comparison

To put the compression gains of this library in context, here is a comparison to the default ROS image compression and the uncompressed images.

![Alt text](https://github.com/nilsbore/data_compression/blob/hydro-devel/data/compression_comparison.png "Compression rates for the different compressed and uncompressed image representation.")

Both the ffv1 codec and the theora codec takes up quite a lot of CPU when compressing the OpenNI streams at 30 fps. The following CPU usages were measured on my Core i7 laptop.

![Alt text](https://github.com/nilsbore/data_compression/blob/hydro-devel/data/cpu_usage_comparison.png "CPU Usage of ffv1 and theora codec on my machine.")
