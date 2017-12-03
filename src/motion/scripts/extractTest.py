#!/usr/bin/env python
import rospy
from motion.srv import essentialEstimation, essentialEstimationRequest, essentialEstimationResponse
from vSLAM_utils.msg import imageFrame,stereoFrame,RMatrix

from geometry_msgs.msg import Point
from std_msgs.msg import String

import transforms3d as t3d
import numpy as np
import copy

rospy.init_node("tester")

##gen world coordinates
x_mean=0
x_dev=4
y_mean=4
y_dev=1
z_mean=10
z_dev=4
nPoints=100

xList=np.random.normal(x_mean,x_dev,nPoints)
yList=np.random.normal(y_mean,y_dev,nPoints)
zList=abs(np.random.normal(z_mean,z_dev,nPoints))

roll=np.random.normal(0,15,1)
pitch=np.random.normal(0,10,1)
yaw=np.random.normal(0,10,1)

tx=np.random.normal(0,1,1)
ty=np.random.normal(0,1,1)
tz=abs(np.random.normal(0,5,1))

print("roll "+str(roll))
print("pitch "+str(pitch))
print("yaw "+str(yaw))

print("x "+str(tx))
print("y "+str(ty))
print("z "+str(tz))

R=t3d.euler.euler2mat(np.pi*roll/180.0,np.pi*pitch/180.0,np.pi*yaw/180.0)
print(R)


homog=np.eye(4,4)
homog[0][0:3]=copy.deepcopy(R[0][:])
homog[1][0:3]=copy.deepcopy(R[1][:])
homog[2][0:3]=copy.deepcopy(R[2][:])
homog[0][3]=tx
homog[1][3]=ty
homog[2][3]=tz
print(homog)

##generate Camera Coordinates

K=np.zeros(shape=(3,3))
K[0][0]=300
K[0][2]=5
K[1][1]=300
K[1][2]=8
K[2][2]=1


P=np.zeros(shape=(3,4))
P[0:3,0:3]=K

print("K" +str(K))
print(P)

## generate points and messages

currentFrame=stereoFrame()
prevFrame=stereoFrame()


setPoints=[]
transformedPoints=[]

projL=[]
projR=[]


for pointIndex in range(0,nPoints):
	#create left and right 3D points relative to respective frames
	tempPoint= np.ones(shape=(4,1))
	tempPoint[0][0]=xList[pointIndex]
	tempPoint[1][0]=yList[pointIndex]
	tempPoint[2][0]=zList[pointIndex]
	setPoints.append(tempPoint)
	newPoint=np.dot(homog,tempPoint)
	transformedPoints.append(newPoint)
	leftPoint=np.dot(P,tempPoint)
	rightPoint=np.dot(P,newPoint)
	projL.append(leftPoint)
	projR.append(rightPoint)
	##pack into message
	#pack left image info
	leftMsg=Point()
	leftMsg.x=leftPoint[0][0]
	leftMsg.y=leftPoint[1][0]
	leftMsg.z=leftPoint[2][0]

	leftlandmarkMsg=Point()
	leftlandmarkMsg.x=tempPoint[0][0]
	leftlandmarkMsg.y=tempPoint[1][0]
	leftlandmarkMsg.z=tempPoint[2][0]

	rightMsg=Point()
	rightMsg.x=rightPoint[0][0]
	rightMsg.y=rightPoint[1][0]
	rightMsg.z=rightPoint[2][0]

	rightlandmarkMsg=Point()
	rightlandmarkMsg.x=newPoint[0][0]
	rightlandmarkMsg.y=newPoint[1][0]
	rightlandmarkMsg.z=newPoint[2][0]
	
	currentFrame.left.features.append(leftMsg)
	currentFrame.left.landmarks.append(leftlandmarkMsg)
	currentFrame.right.features.append(leftMsg)
	currentFrame.right.landmarks.append(leftlandmarkMsg)

	prevFrame.left.features.append(rightMsg)
	prevFrame.left.landmarks.append(rightlandmarkMsg)
	prevFrame.right.features.append(rightMsg)
	prevFrame.right.landmarks.append(rightlandmarkMsg)

	


rospy.wait_for_service("extract/EssentialMotion",5)
serv=rospy.ServiceProxy("extract/EssentialMotion",essentialEstimation)

req=essentialEstimationRequest()
req.current=currentFrame
req.previous=prevFrame

for index in range(0,4):
	print(req.current.left.landmarks[index])
	print(req.previous.left.landmarks[index])

answer=serv(req)
print(answer)


