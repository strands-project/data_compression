#!/usr/bin/env python
import sched, time, os, sys
#import shutil # for debugging, saving the original images

def callback(sc, impath, nbr):
    sc.enter(20, 1, callback, (sc, impath, nbr+1))
    print "Compressing..."
    temps = []
    counter = 0
    flist = os.listdir(impath)
    flist = filter(lambda s: s[:5] == "depth", flist)
    flist.sort(key = lambda s: int(s[-10:-4]))
    for f in flist:
        if not os.path.isfile(os.path.join(impath, f)): # should not happen
            continue
        #if f[:5] != "depth":
            #continue
        depthtemp = os.path.join(impath, "tempdepth%06d.png" % counter)
        rgbtemp = os.path.join(impath, "temprgb%06d.png" % counter)
        rgbf = "rgb" + f[-10:-4] + ".png"
        os.rename(os.path.join(impath, f), depthtemp) # check if there really are depths coming in
        os.rename(os.path.join(impath, rgbf), rgbtemp) # check if there really are rgbs coming in
        #shutil.copy(depthtemp, os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, "debug", f))) # for debugging, saving the original images
        #shutil.copy(rgbtemp, os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, "debug", rgbf))) # for debugging, saving the original images
        temps.append(depthtemp)
        temps.append(rgbtemp)
        counter += 1
    depthimages = os.path.join(impath, "tempdepth%06d.png")
    rgbimages = os.path.join(impath, "temprgb%06d.png")
    avconv = os.path.abspath(os.path.join(os.path.expanduser('~'), "libav", "bin", "avconv"))
    depthvideo = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, "videos", "depth%06d.mov" % nbr))
    rgbvideo = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, "videos", "rgb%06d.mov" % nbr)) # maybe this has to be mkv??
    os.system("%s -r 30 -i %s -pix_fmt gray16 -vsync 1 -vcodec ffv1 -coder 1 %s" % (avconv, depthimages, depthvideo))
    os.system("%s -r 30 -i %s -c:v libx264 -preset ultrafast -crf 0 %s" % (avconv, rgbimages, rgbvideo))
    for f in temps:
        os.remove(f)

def batch_compressor(argv):
    s = sched.scheduler(time.time, time.sleep)
    nbr = 0
    s.enter(20, 1, callback, (s, argv, nbr))
    s.run()

if __name__ == "__main__":
    batch_compressor(sys.argv[1])
