#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2,cv_bridge
import numpy as np

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window",1)
        self.image_sub =rospy.Subscriber('camera/rgb/image_raw',
                                          Image,self.image_callback)

    def image_callback(self,msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #print image

        cv2.imshow("window",image)
        cv2.waitKey(1)

rospy.init_node('follower')

follower = Follower()

rospy.spin()




