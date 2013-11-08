data_compression
================

Video encoding for 8 bit RGB images, 16 bit grayscale depth images and possibly more. The package is laid out as a catkin package.

Here you can find information for installing libav 9, which is required to compile this: https://github.com/strands-project/data_compression/wiki/Setup-libav

Here is the notes on how to use the launch files to compress and decompress the incoming openni_launch topics (there is also stuff for evaluating the compression):
https://github.com/strands-project/data_compression/wiki/Using-the-nodes-and-launch-files

And here is instructions for using the libav command line tool to losslessly compress depth images: https://github.com/strands-project/data_compression/wiki/Use-avconv-to-losslessly-compress-depth-video

# strandsbag package
This package aims to replicate the rosbag functionality but with video compression of image topics. It has a couple of launch files: `record.launch` and `play.launch`. play uses `image_player_node` which reads a directory and outputs them at the correct ros time by looking at their timestamps.
   * `record.launch` takes as input arg `bag_folder`, which is the empty folder where we store the rosbag and videos. Note that you have to create this before launching. It will start a rosbag record together with video compression of image topics.
   * `play.launch` also takes as input `bag_folder`, also the path to the folder containing the rosbag and videos. The node starts rosbag play with a simulated time to allow image_player_node to synch the image outputs. Before you launch this node, you need to run `rosrun libav_compressor video_decompressor.py _folder:=(BAG_FOLDER)`.

# openni_saver package
   * This package contains one node, `openni_saver_node`. It saves images in 1x16 bit depth format and 3x8 bit rgb format respectively. It subscribes to the ros image topics from `openni_launch openni.launch`. Every time a folder is done saving it calls the `CompressorService` if available.
   * The openni_saver_node offers the `LoggingService` ros service. It can be called with three different actions: `start`, `stop` and `new`. Start is meant to start a new recording at a waypoint, with the ability to compile all images into one video with a specific name. This name can be specified in the `folder` argument. stop stops the recording at the waypoint. new should not be used with rosbag but adds the ability to store the images in a new parent folder.

# libav_compressor package
This package contains two nodes, `online_compressor` and `video_decompressor`. They are most easily used through one of the launch files. The online_compressor waits for calls by image_saver_node to compile the latest images in a specified folder. video_decompressor decompresses these images again.
   * `video_saver.launch` launches openni_saver_node together with the online_compressor. It compresses the images. Make sure you have launched openni.launch before running this.

# hints
When you use rviz to display the recreated images, for example using the play_images node, it's important that you uncheck the Normalize Range box. Then set the min value to 0 and the max value to 2^12=4098.
