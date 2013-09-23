#!/usr/bin/env python
import os, sys

def remove_files(folder):
    flist = os.listdir(folder)
    for f in flist:
        os.remove(os.path.join(folder, f))
    
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "Not enough arguments to remove_files.py"
    else:
        remove_files(sys.argv[1])
