#!/usr/bin/env	python
import glob

import cv2
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image

rospy.init_node('read_dino')

rate=rospy.Rate(15)

TheDir = "../../../media/yasin/0EFE2BA4FE2B8357/Flash/Master/dino/"
image_names = glob.glob(TheDir+"/*.png")
image_names = sorted(image_names,key=lambda x: int(x[len(TheDir)+4:-4]))

class Reader:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window",1)
        self.images_pup =rospy.Publisher('/cv_camera/image_raw',Image,queue_size= 1)


    def ImgPub(self,img):
        I = cv2.imread(img,1)
        imagemsg = self.bridge.cv2_to_imgmsg(I)
        imagemsg.encoding = 'bgr8'
        self.images_pup.publish(imagemsg)

c = Reader()
for img in image_names:
    c.ImgPub(img)
    print("Image is passed " + img)
    rate.sleep()
