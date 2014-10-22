rosbag_openni_compression
=========================

To use this, you may have to clone https://github.com/ros-perception/image_transport_plugins/tree/hydro-devel.
The apt-get package did not work for me for some reason. You also have to get https://github.com/nilsbore/libav_image_transport/tree/catkinized_libav, specifically the branch `catkinized_libav`. 

At the moment, you have to specify one camera to compress,
either `head_xtion` or `chest_xtion`, with the `camera` parameter to `record.launch`. The commands are

* `roslaunch rosbag_openni_compression record.launch file:=/path/to/file.bag camera:=head_xtion`

to record a rosbag in the file `/path/to/file.bag` and

* `roslaunch rosbag_openni_compression play.launch file:=/path/to/file.bag camera:=head_xtion`

to play the file back, together with reconstruction of all OpenNI images and pointclouds. There is nothing limiting this to just one camera except the launch files atm.
