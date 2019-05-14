#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2,cv_bridge
import numpy as np

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow('pic', cv2.WINDOW_NORMAL)
        self.image_sub =rospy.Subscriber('/camera/rgb/image_raw',
                                          Image,self.image_callback)
#        self.image_sub =rospy.Subscriber('/cv_camera/image_raw',
#                                          Image,self.image_callback)


    def image_callback(self,msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #print image.shape#(480,360)
        cv2.namedWindow('pic', cv2.WINDOW_NORMAL)
        cv2.imshow("pic",image)
        #print(np.shape(image))
        cv2.waitKey(1)

rospy.init_node('reader')

follower = Follower()

rospy.spin()





