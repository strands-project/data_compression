#!/usr/bin/env python
import sched, time, os, sys

def batch_decompressor(impath):
    print "Decompressing..."
    vidpath = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, "videos"))
    counter = 0
    flist = os.listdir(vidpath)
    flist = filter(lambda s: s[:5] == "depth", flist)
    flist.sort(key = lambda s: int(s[5:14]))
    for f in flist:
        depthvid = os.path.join(vidpath, f)
        if not os.path.isfile(depthvid):
            continue
        rgbvid = os.path.join(vidpath, "rgb" + f[5:14] + ".mov") # should this be mkv?
        timef = open(os.path.join(vidpath, "time" + f[5:14] + ".txt"), 'r')
        depthimages = os.path.join(impath, "tempdepth%06d.tiff")
        rgbimages = os.path.join(impath, "temprgb%06d.png")
        avconv = os.path.abspath(os.path.join(os.path.expanduser('~'), "libav", "bin", "avconv"))
        os.system("%s -i %s -f image2 -pix_fmt gray16 %s" % (avconv, depthvid, depthimages))
        os.system("%s -i %s -f image2 %s" % (avconv, rgbvid, rgbimages))
        templist = os.listdir(impath)
        templist = filter(lambda s: s[:9] == "tempdepth", templist)
        templist.sort(key = lambda s: int(s[-11:-5]))
        for t in templist:
            time = timef.readline()
            rgbt = os.path.join(impath, "temprgb" + t[-11:-5] + ".png")
            os.rename(os.path.join(impath, t), os.path.join(impath, "depth%s.tiff" % time))
            os.rename(rgbt, os.path.join(impath, "rgb%s.png" % time))
            counter += 1
        timef.close()

if __name__ == "__main__":
    batch_decompressor(sys.argv[1])
