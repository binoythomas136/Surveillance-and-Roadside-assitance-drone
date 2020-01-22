#!/usr/bin/env python

from std_msgs.msg import String
import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped,Vector3Stamped
import numpy as np
from math import *
import math
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamSet
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate, StreamRateRequest
from sensor_msgs.msg import NavSatFix
import time
from tf.transformations import euler_from_quaternion, euler_matrix, quaternion_from_euler
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import rospy
import cv2
#from __future__ import print_function
import numpy as np
import roslib
from subscribers import Subscribers



class RoadFollow():
	
	def color(self,image):
	    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV).astype(np.float)
	    # lower mask (0-15)
	    lower_red = np.array([0,5,0])
	    upper_red = np.array([15,255,255])
	    mask0 = cv2.inRange(hsv, lower_red, upper_red)

	    # upper mask (170-180)
	    lower_red = np.array([155,5,0])
	    upper_red = np.array([179,255,255])
	    mask1 = cv2.inRange(hsv, lower_red, upper_red)

	    mask = mask0+mask1
	    kernel = np.ones((3,3),np.uint8)
	    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
	    cv2.imwrite('Images123/road_closing.jpg', closing)
	    return closing

	def topPixel(self,image):
	    for i in range(image.shape[0]/4):
		mean = 0
		sum = 0
		midPoint = int(np.floor(image.shape[1]/2))
		countLeft = np.sum(image[i,0:midPoint])/255
		countRight = np.sum(image[i,midPoint:])/255
		if(int(countLeft) > 10 and int(countRight) > 10):
		    count = countLeft + countRight
		    for j in range(image.shape[1]):
		        if(image[i,j]==255):
		            sum = sum + j
		    mean = sum/np.floor(count)
		    if(int(mean) > 0):
		        return (i,int(mean))
		elif(int(countLeft) > 10):
		    count = countLeft
		    for j in range(image.shape[1]):
		        if(image[i,j]==255):
		            sum = sum + j
		    mean = sum/np.floor(count)
		    if(int(mean) > 0):
		        return (i,int(mean)-20)
		elif(int(countRight) > 10):
		    count = countRight
		    for j in range(image.shape[1]):
		        if(image[i,j]==255):
		            sum = sum + j
		    mean = sum/np.floor(count)
		    if(int(mean) > 0):
		        return (i,int(mean)+20)
	    return (1000,1000)

	def getOrientation(self,image,posx,posy):
	    if(posx==1000):
		return 1000
	    midx = image.shape[0]
	    midy = np.ceil(image.shape[1]/2)
	    a = midx-posx-1
	    b = midy-posy-1
	    angle = math.degrees(math.atan(b/a))
	    return angle


	def getRoadAngle(self,frame):
	    print('here')
	    image = np.copy(frame)
	    rgb = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
	    mask = self.color(image)
	    posx,posy = self.topPixel(mask)
	    angle = self.getOrientation(rgb,posx,posy)
	    
	    return angle
