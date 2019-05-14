#!/usr/bin/env	python
import rospy
import numpy as np

from geometry_msgs.msg import Pose,Point
from visualization_msgs.msg import Marker
from read_poses_dinoSR import Rs,Ts

rospy.init_node('read_poses')

take7,pose,all_vec_pose=0,[],[]
#Cams = PoseArray()
#Cams.header.frame_id = "world"
#Cams.header.stamp = rospy.Time.now();

# Reading the Files
with open("nodes_and_prjcts.txt") as f:
    for line in f:
        if take7:
            pose +=[float(line)]
            take7-=1
        if 'P' in line:
            take7=7
            if pose:
                all_vec_pose+=[np.array(pose)]
            pose=[]
if pose:
    all_vec_pose+=[np.array(pose)]

def add_cor_for_pose(i,points=[]):

    #if np.any(pos):pos = [0,0,0,1,0,0,0]
        #markr.color.r=1.0
        #markr.color.b=0.0
    r,t = Rs[i].T,Ts[i]
    a,b,c = -r.dot(t)
    H = np.hstack((r,[a,b,c]))
    H = np.vstack((H,[0,0,0,1]))    
    print(a,b,c)
    #H = H.dot(H_c)
    #TODO calculate Transform Matrix
    #a,b,c = -np.array([ x[:-1] for x in H[:-1]]).dot(np.array([[a],[b],[c]]))
    p = Point(a,b,c)
    pt = Point(c,-a,-b)

    points.append(pt)
    #Z axis
    z = H.dot(np.array([0,0,0.3,1.0]))
    z = z/float(z[-1])
    opt = Point(z[0],z[1],z[2])
    opt = Point(z[2],-z[0],-z[1])
    points.append(opt)

    points.append(pt)
    #X axis
    x = H.dot(np.array([0.2,0,0,1]))
    x = x/float(x[-1])
    opt = Point(x[0],x[1],x[2])
    opt = Point(x[2],-x[0],-x[1])
    points.append(opt)

    points.append(pt)
    #Y axis
    y = H.dot(np.array([0,0.1,0,1]))
    y = y/float(y[-1])
    opt = Point(y[0],y[1],y[2])
    opt = Point(y[2],-y[0],-y[1])
    points.append(opt)
    return points


#Cam_Poses =rospy.Publisher('Cameras',	PoseArray,queue_size= 1)
Cam_Poses =rospy.Publisher('Original_Cameras',Marker,queue_size= 1)
rate=rospy.Rate(2)

Cameras = Marker()
Cameras.header.frame_id = "pgraph"
Cameras.header.stamp = rospy.Time.now()
Cameras.ns = "pgraph"
Cameras.id = 0
Cameras.type = 5
Cameras.action = 0
Cameras.scale.x = .02
Cameras.scale.y = .02
Cameras.scale.z = .02
Cameras.color.b = 1.0
Cameras.color.a = 1.0
Cameras.lifetime = rospy.Duration()

Cameras.pose.position.x = 0
Cameras.pose.position.y = 0
Cameras.pose.position.z = 0
Cameras.pose.orientation.w = 0.0
Cameras.pose.orientation.x = 0.0
Cameras.pose.orientation.y = 0.0
Cameras.pose.orientation.z = 1.0

points = []
while not rospy.is_shutdown():
    for i,p in enumerate(all_vec_pose[0:-1:1]):
        #postam = PoseStamped()
        points = add_cor_for_pose(i,points)
        #points = add_cor_for_pose(0,points)
    Cameras.points = points
    points = []
    Cam_Poses.publish(Cameras)
    rate.sleep()
