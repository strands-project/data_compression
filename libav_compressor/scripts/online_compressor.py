#!/usr/bin/env python
import sched, time, os, sys
import shutil # for debugging, saving the original images
import rospy
from libav_compressor.srv import CompressionService
from std_msgs.msg import String
from threading import Thread, Lock
from Queue import Queue

def compress_folder(result_queue, folder):
    rgbimages = os.path.join(folder, "rgb%06d.png")
    depthimages = os.path.join(folder, "depth%06d.png")
    
    rgbvideo = os.path.join(folder, "rgb.mov")
    depthvideo = os.path.join(folder, "depth.mkv")
    
    avconv = os.path.abspath(os.path.join(os.path.expanduser('~'), "libav", "bin", "avconv"))
    
    # compress depth video
    os.system("%s -r 30 -i %s -pix_fmt gray16 -vsync 1 -vcodec ffv1 -coder 1 %s" % (avconv, depthimages, depthvideo))
    # compress rgb video
    os.system("%s -r 30 -i %s -c:v libx264 -preset ultrafast -crf 0 %s" % (avconv, rgbimages, rgbvideo))
    
    flist = os.listdir(folder)
    
    for f in flist:
        if len(f) > 10:
            os.remove(os.path.join(folder, f))
    
    result_queue.put(folder)

class online_compressor():

    def __init__(self):
        self.queue = Queue()
        # not sure about the name
        srv = rospy.Service('compression_service', CompressionService, self.compression_service)
        # Create a publisher that sends message when compression done
        pub = rospy.Publisher("compression_done", String)
        while not rospy.is_shutdown():
            if not self.queue.empty():
                val = self.queue.get()
                pub.publish(val)
                print "Publishing %s" % val
            else: # remove
                print 'empty'
            rospy.sleep(1.0)
    
    def compression_service(self, req):
        folder = req.folder
        thread = Thread(target = compress_folder, args=(self.queue, folder))
        thread.start()
        return True
        
    def create_folder(self, path):
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
