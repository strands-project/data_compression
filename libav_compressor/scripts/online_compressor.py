#!/usr/bin/env python
import sched, time, os, sys
import shutil # for debugging, saving the original images
import rospy
from libav_compressor.srv import CompressionService
from std_msgs.msg import String
from threading import Thread, Lock
from Queue import Queue

def compress_folder(result_queue, folder):
    rgbimages = os.path.join(folder, "rgb%06d.png") # name structure of images to be compressed
    depthimages = os.path.join(folder, "depth%06d.png")
    
    rgbvideo = os.path.join(folder, "rgb.mov") # resulting rgb video
    depthvideo = os.path.join(folder, "depth.mkv") # resulting depth video
    
    # the install path of avconv, defaults to ~/libav/bin/avconv, should be argument
    avconv = os.path.abspath(os.path.join(os.path.expanduser('~'), "libav", "bin", "avconv"))
    
    # compress depth video
    os.system("%s -r 30 -i %s -pix_fmt gray16 -vsync 1 -vcodec ffv1 -coder 1 %s" % (avconv, depthimages, depthvideo))
    # compress rgb video
    os.system("%s -r 30 -i %s -c:v libx264 -preset ultrafast -crf 0 %s" % (avconv, rgbimages, rgbvideo))
    
    flist = os.listdir(folder)
    
    # remove compressed images
    for f in flist:
        if len(f) > 10:
            os.remove(os.path.join(folder, f))
    
    result_queue.put(folder) # put the folder name on the completed queue

class online_compressor():

    def __init__(self):
        self.queue = Queue() # result queue, folder names are put here when complete
        srv = rospy.Service('compression_service', CompressionService, self.compression_service)
        # Create a publisher that sends message when compression done
        pub = rospy.Publisher("compression_done", String)
        while not rospy.is_shutdown():
            # no locks needed, queue manages that
            if not self.queue.empty(): # check if any were compressed
                val = self.queue.get() # get latest compressed folder
                pub.publish(val)
                print "Publishing %s" % val
            rospy.sleep(1.0)
    
    def compression_service(self, req): # start a new compression thread
        folder = req.folder
        thread = Thread(target = compress_folder, args=(self.queue, folder))
        thread.start() # no need to manage threads in python, automatic gc
        return True
        
    def create_folder(self, path): # not really needed anymore
        try:
            os.makedirs(path)
        except OSError:
            pass

if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node('online_compressor')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        compressor = online_compressor()
    except rospy.ROSInterruptException:
        pass
