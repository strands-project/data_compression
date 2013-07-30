#!/usr/bin/env python
import sched, time, os, sys
#import shutil # for debugging, saving the original images

def callback(sc, impath, nbr):
    sc.enter(20, 1, callback, (sc, impath, nbr+1))
    print "Compressing..."
    temps = []
    counter = 0
    flist = os.listdir(impath)
    flist.sort(key = lambda s: (s[:-10], int(s[-10:-4])))
    for f in flist:
        if not os.path.isfile(os.path.join(impath, f)):
            continue
        if f[:5] != "depth":
            continue
        tempname = os.path.join(impath, "tempdepth%06d.png" % counter)
        os.rename(os.path.join(impath, f), tempname)
        #shutil.copy(tempname, os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, "debug", f))) # for debugging, saving the original images
        temps.append(tempname)
        counter += 1
    depthimages = os.path.join(impath, "tempdepth%06d.png")
    avconv = os.path.abspath(os.path.join(os.path.expanduser('~'), "libav", "bin", "avconv"))
    video = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, "videos", "depth%06d.mov" % nbr))
    os.system("%s -r 30 -i %s -pix_fmt gray16 -vsync 1 -vcodec ffv1 -coder 1 %s" % (avconv, depthimages, video))
    for f in temps:
        os.remove(f)

def batch_compressor(argv):
    s = sched.scheduler(time.time, time.sleep)
    nbr = 0
    s.enter(20, 1, callback, (s, argv, nbr))
    s.run()

if __name__ == "__main__":
    batch_compressor(sys.argv[1])
