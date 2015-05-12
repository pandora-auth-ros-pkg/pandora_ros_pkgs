#!/usr/bin/python

PKG = 'pandora_vision_annotator' # this package name

import time, sys, os
from ros import rosbag
import roslib; roslib.load_manifest('sensor_msgs')
import rospy
from sensor_msgs.msg import Image

import ImageFile

def GetFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    print( "Searching directory %s" % dir )
    all = []
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg', '.ppm']:
                    all.append( os.path.join( path, f ) )
    return all


def CreateBag(imgs,bagname):
    '''Creates a bag file with camera images'''
    bag =rosbag.Bag(bagname, 'w')

    try:
        for i in range(len(imgs)):
            print("Adding %s" % imgs[i])
            fp = open( imgs[i], "r" )
            p = ImageFile.Parser()

            while 1:
                s = fp.read(1024)
                if not s:
                    break 
                p.feed(s)

            im = p.close()
            if im.mode != "RGB":
                im=im.convert("RGB")
            Stamp = rospy.rostime.Time.from_sec(time.time())
            Img = Image()
            Img.header.stamp = Stamp
            Img.width = im.size[0]
            Img.height = im.size[1]
            Img.encoding = "rgb8"
            Img.step = Img.width *3;
            Img.header.frame_id = "camera"
            Img_data = [pix for pixdata in im.getdata() for pix in pixdata]
            Img.data = Img_data

            bag.write('/camera/image', Img, Stamp)
    finally:
        bag.close()       

if __name__ == "__main__":
    if len( sys.argv ) == 3:
        all_imgs = GetFilesFromDir(sys.argv[1])
        if len(all_imgs) <= 0:
            print("No images found in %s" % sys.argv[1])
            exit()
        CreateBag(all_imgs, sys.argv[2])
    else:
        print( "Usage: img2bag imagedir bagfilename")
