#!/usr/bin/env python
import rospy
from vSLAM_utils.srv import extractMotion, extractMotionRequest, extractMotionResponse
import transforms3d as t3d
import numpy as np
import copy

rospy.init_node("tester")

##gen world coordinates
x_mean=5
x_dev=4
y_mean=1
y_dev=3
z_mean=0
z_dev=8
nPoints=100

xList=np.random.normal(x_mean,x_dev,nPoints)
yList=np.random.normal(y_mean,y_dev,nPoints)
zList=abs(np.random.normal(z_mean,z_dev,nPoints))

roll=np.random.normal(0,15,1)
pitch=np.random.normal(0,10,1)
yaw=np.random.normal(0,10,1)

tx=abs(np.random.normal(0,5,1))
ty=np.random.normal(0,2,1)
tz=np.random.normal(0,2,1)

print("roll "+str(roll))
print("pitch "+str(pitch))
print("yaw "+str(yaw))

print("x "+str(tx))
print("y "+str(ty))
print("z "+str(tz))

R=t3d.euler.euler2mat(np.pi*roll/180.0,np.pi*pitch/180.0,np.pi*yaw/180.0)
print(R)


homog=np.eye(4,4)
print(homog[0][0:3])
print(R[0][:])
homog[0][0:3]=copy.deepcopy(R[0][:])
homog[1][0:3]=copy.deepcopy(R[1][:])
homog[2][0:3]=copy.deepcopy(R[2][:])
homog[0][3]=tx
homog[1][3]=ty
homog[2][3]=tz
print(homog)


rospy.wait_for_service("/extract/motion",5)
serv=rospy.ServiceProxy("/extract/motion",extractMotion)


rospy.spin()
