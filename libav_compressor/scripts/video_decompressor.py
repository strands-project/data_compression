#!/usr/bin/env python
import os, sys
import rospy

class video_decompressor():

    def decompress_folder(self, folder):
        # image names, numbered from 1
        rgbimages = os.path.join(folder, "rgb%06d.png")
        depthimages = os.path.join(folder, "depth%06d.tiff")
        
        # video names
        rgbvideo = os.path.join(folder, "rgb.mov")
        depthvideo = os.path.join(folder, "depth.mkv")
        
        # decompress the depth video
        if os.path.isfile(depthvideo):
            os.system("%s -i %s -f image2 -pix_fmt gray16 %s" % (self.avconv, depthvideo, depthimages))
        # decompress the rgb video
        if os.path.isfile(rgbvideo):
            os.system("%s -i %s -f image2 %s" % (self.avconv, rgbvideo, rgbimages))

    def __init__(self):
        folder = rospy.get_param("~folder", 'default')
        dlist = os.listdir(folder)
        # path to the install of libav
        self.avconv = os.path.abspath(os.path.join(os.path.expanduser('~'), "libav", "bin", "avconv"))
        # just go through the folders, decompress them if videos exist
        for d in dlist:
            dpath = os.path.join(folder, d)
            if os.path.isdir(dpath):
                self.decompress_folder(dpath)

if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node('video_decompressor')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        decompressor = video_decompressor()
    except rospy.ROSInterruptException:
        pass
