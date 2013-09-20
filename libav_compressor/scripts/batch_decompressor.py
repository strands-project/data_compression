#!/usr/bin/env python
import sched, time, os, sys

def batch_decompressor(impath, vidpath):
    print "Decompressing..."
    flist = os.listdir(vidpath) # find all depth videos, order them
    flist = filter(lambda s: s[:5] == "depth", flist)
    flist.sort(key = lambda s: int(s[5:15]))
    for f in flist:
        depthvid = os.path.join(vidpath, f) # depth video path
        if not os.path.isfile(depthvid):
            continue
        # the rgbvids are named similarly
        rgbvid = os.path.join(vidpath, "rgb" + f[5:15] + ".mov") # should this be mkv?
        # load the timestamps text file
        timef = open(os.path.join(vidpath, "time" + f[5:15] + ".txt"), 'r')
        # formatting of temporary images
        depthimages = os.path.join(impath, "tempdepth%06d.tiff")
        rgbimages = os.path.join(impath, "temprgb%06d.png")
        # the path to the libav convenience tool
        avconv = os.path.abspath(os.path.join(os.path.expanduser('~'), "libav", "bin", "avconv"))
        # decompress the depth video
        os.system("%s -i %s -f image2 -pix_fmt gray16 %s" % (avconv, depthvid, depthimages))
        # decompress the rgb video
        os.system("%s -i %s -f image2 %s" % (avconv, rgbvid, rgbimages))
        templist = os.listdir(impath) # list temporary depth images
        templist = filter(lambda s: s[:9] == "tempdepth", templist)
        templist.sort(key = lambda s: int(s[-11:-5]))
        for t in templist:
            time = timef.readline() # the depth image filename
            rgbt = os.path.join(impath, "temprgb" + t[-11:-5] + ".png")
            os.rename(os.path.join(impath, t), os.path.join(impath, "%s.tiff" % time[:-1]))
            time = timef.readline() # the rgb image filename
            os.rename(rgbt, os.path.join(impath, "%s.png" % time[:-1]))
        timef.close()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print "Not enough arguments to batch_decompressor.py"
    else:
        batch_decompressor(sys.argv[1], sys.argv[2])
