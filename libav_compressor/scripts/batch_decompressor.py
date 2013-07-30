#!/usr/bin/env python
import sched, time, os, sys

def batch_decompressor(impath):
    print "Decompressing..."
    vidpath = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, "videos"))
    counter = 0
    flist = os.listdir(vidpath)
    flist.sort(key = lambda s: (s[:-10], int(s[-10:-4])))
    for f in flist:
        vidfile = os.path.join(vidpath, f)
        if not os.path.isfile(vidfile):
            continue
        if f[:5] != "depth":
            continue
        depthimages = os.path.join(impath, "tempdepth%06d.tiff")
        avconv = os.path.abspath(os.path.join(os.path.expanduser('~'), "libav", "bin", "avconv"))
        os.system("%s -i %s -f image2 -pix_fmt gray16 %s" % (avconv, vidfile, depthimages))
        templist = os.listdir(impath)
        templist = filter(lambda s: s[:4] == "temp", templist)
        templist.sort(key = lambda s: int(s[-11:-5]))
        for t in templist:
            os.rename(os.path.join(impath, t), os.path.join(impath, "depth%06d" % counter))
            counter += 1

if __name__ == "__main__":
    batch_decompressor(sys.argv[1])
