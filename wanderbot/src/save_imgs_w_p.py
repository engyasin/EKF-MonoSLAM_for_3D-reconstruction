#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import cv2,cv_bridge
import numpy as np

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window",1)
        self.image_sub =rospy.Subscriber('/camera/rgb/image_raw',
                                          Image,self.image_callback)
#        self.image_sub =rospy.Subscriber('/cv_camera/image_raw',
#                                          Image,self.image_callback)
        self.odom_sub = rospy.Subscriber('/odom',Odometry,self.odom_callback)
        self.take_img_every = 10 # take frame each 10 ones (6 Hz) make it 1.4 s late
        self.counter = 0
        self.Poses_file = open("cs_and_qs.txt","w")
        self.P = ['','']


    def image_callback(self,msg):
        if (self.counter%self.take_img_every) == 0:
            #write the pose first
            self.Poses_file.writelines("P"+str(self.counter) +"\n")
            self.Poses_file.writelines(self.P[0])
            self.Poses_file.writelines("\n")
            self.Poses_file.writelines(self.P[1])
            self.Poses_file.writelines("\n")

            image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            #print image.shape#(480,360)

            #cv2.imshow("window",image)
            cv2.imwrite(str(self.counter)+".jpg",image)
            #cv2.waitKey(1)


        self.counter += 1

    def odom_callback(self,msg):
        A1 = msg.pose.pose.position
        self.P[0] = ' '.join(map(str,[A1.x,A1.y,A1.z]))
        A2 = msg.pose.pose.orientation
        self.P[1] = ' '.join(map(str,[A2.x,A2.y,A2.z,A2.w]))

    def __del__(self):
        self.Poses_file.close()
        print("file closed correctly")


rospy.init_node('subsampler')

follower = Follower()

rospy.spin()

del follower



