mongodb_openni_compression
==========================

Video encoding for OpenNI topics to record them to a mongodb_store, https://github.com/strands-project/mongodb_store.

Launch an action server that records video on demand by:

`roslaunch mongodb_openni_compression record_server.launch`

Then start the recording by calling the action server

`/record_camera`, e.g. by `rosrun actionlib axclient.py /record_camera`

Supply the arguments `camera`, the namespace of the OpenNI camera used, e.g. `head_xtion`. Also supply `with_depth`, `with_rgb`,
bool arguments for whether to record the depth and rgb.

If you supply `head_xtion` as camera namespace, the depth topics will be saved as `/head_xtion_compressed_depth_libav`
in the mongodb datacentre, in the `roslog` collection. The rgb is saved as `/head_xtion_compressed_rgb_compressed`.

If you want to replay them, use the `mongodb_play.py` utility:

`rosrun mongodb_store mongodb_play.py /head_xtion_compressed_rgb_compressed /head_xtion_compressed_depth_libav`

You can use `image_transport republish` to convert the images back to raw format, or view the compressed
streams directly in Rviz, by choosing either `libav` or `theora` as the transport hint.
To convert the streams (assuming camera namespace was set to `head_xtion`:

* `rosrun image_transport republish libav in:=/head_xtion_compressed_depth/libav raw out:=/head_xtion/depth/image_raw`
* `rosrun image_transport republish theora in:=/head_xtion_compressed_rgb/theora raw out:=/head_xtion/rgb/image_raw`
