'''
Source: https://answers.ros.org/question/11537/creating-a-bag-file-out-of-a-image-sequence/
(plus some additions from the comment section)
'''

import time, sys, os
from ros import rosbag
import roslib
import rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image

import ImageFile

def CompSortFileNamesNr(f):
    g = os.path.splitext(os.path.split(f)[1])[0]
    numbertext = ''.join(c for c in g if c.isdigit())
    return int(numbertext)

def GetFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    print( "Searching directory %s" % dir )
    all = []
    left_files = []
    right_files = []
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in sorted(files, key=CompSortFileNamesNr):
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg', '.ppm']:
                    if 'left' in f or 'left' in path:
                        left_files.append( os.path.join( path, f ) )
                    elif 'right' in f or 'right' in path:
                        right_files.append( os.path.join( path, f ) )
                    all.append( os.path.join( path, f ) )
    return all, left_files, right_files

def CreateStereoBag(left_imgs, right_imgs, bagname):
    '''Creates a bag file containing stereo image pairs'''
    bag =rosbag.Bag(bagname, 'w')

    try:
        for i in range(len(left_imgs)):
            print("Adding %s" % left_imgs[i])
            fp_left = open( left_imgs[i], "r" )
            p_left = ImageFile.Parser()

            while 1:
                s = fp_left.read(1024)
                if not s:
                    break
                p_left.feed(s)

            im_left = p_left.close()

            fp_right = open( right_imgs[i], "r" )
            print("Adding %s" % right_imgs[i])
            p_right = ImageFile.Parser()

            while 1:
                s = fp_right.read(1024)
                if not s:
                    break
                p_right.feed(s)

            im_right = p_right.close()

            Stamp = roslib.rostime.Time.from_sec(time.time())

            Img_left = Image()
            Img_left.header.stamp = Stamp
            Img_left.width = im_left.size[0]
            Img_left.height = im_left.size[1]
            Img_left.encoding = "rgb8"
            Img_left.header.frame_id = "camera/left"
            Img_left_data = [pix for pixdata in im_left.getdata() for pix in pixdata]
            Img_left.data = Img_left_data
            Img_right = Image()
            Img_right.header.stamp = Stamp
            Img_right.width = im_right.size[0]
            Img_right.height = im_right.size[1]
            Img_right.encoding = "rgb8"
            Img_right.header.frame_id = "camera/right"
            Img_right_data = [pix for pixdata in im_right.getdata() for pix in pixdata]
            Img_right.data = Img_right_data

            bag.write('camera/left/image_raw', Img_left, Stamp)
            bag.write('camera/right/image_raw', Img_right, Stamp)
    finally:
        bag.close()

def CreateMonoBag(imgs,bagname):
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

            #Stamp = rospy.rostime.Time.from_sec(time.time())
            Stamp = rospy.Time.from_sec(time.time()) 
            Img = Image()
            Img.header.stamp = Stamp
            Img.width = im.size[0]
            Img.height = im.size[1]
            #Img.encoding = "rgb8"
            Img.encoding = "mono8"
            Img.header.frame_id = "camera"
            Img_data = [pix for pixdata in im.getdata() for pix in pixdata]
            #Img_data = [pix for pixdata in [im.getdata()] for pix in pixdata]
            Img.data = Img_data
            #Change "rgb8" encoding to "mono8", and "[pix for pixdata in im.getdata() for pix in pixdata]" to "[pix for pixdata in [im.getdata()] for pix in pixdata]"

            bag.write('camera/image_raw', Img, Stamp)
    finally:
        bag.close()       


def CreateBag(args):
    '''Creates the actual bag file by successively adding images'''
    all_imgs, left_imgs, right_imgs = GetFilesFromDir(args[0])
    if len(all_imgs) <= 0:
        print("No images found in %s" % args[0])
        exit()

    if len(left_imgs) > 0 and len(right_imgs) > 0:
        # create bagfile with stereo camera image pairs
        CreateStereoBag(left_imgs, right_imgs, args[1])
    else:
        # create bagfile with mono camera image stream
        CreateMonoBag(all_imgs, args[1])        

if __name__ == "__main__":
    if len( sys.argv ) == 3:
        CreateBag( sys.argv[1:])
    else:
        print( "Usage: img2bag imagedir bagfilename")
