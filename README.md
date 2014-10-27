data_compression
================

Video encoding for OpenNI topics to record them to a rosbag and replay through reconstruction of point clouds.

Includes libav_image_transport by Dominique Hunziker: https://github.com/rapyuta/libav_image_transport.

## Compiling

Before doing `catkin_make` on the containing workspace, you need to fetch the submodules one time

* `git submodule init`
* `git submodule update`

within the git repository.
