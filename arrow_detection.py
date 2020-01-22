#!/usr/bin/env python

from std_msgs.msg import String
import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped,Vector3Stamped
import numpy as np
from math import sin,cos,radians,atan2,sqrt,pi
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


class Arrow():
	def rotateImage(self,image, angle):
	    image_center = tuple(np.array(image.shape[1::-1]) / 2)
	    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
	    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
	    return result
	def crop_img (self,imgg):
	    p=(cv2.cvtColor(imgg,cv2.COLOR_RGB2GRAY))
	    ind=np.argmax(np.sum(p,axis=1))
	    sum=0
	    count=0
	    mean=0
	    for j in range(0,p.shape[1]):
		if(p[ind,j]!=0):
		    sum = sum + j
		    count = count + 1
		    mean = np.floor(sum/count)
	    box_size=400
	    mean=int(mean)
	    if(ind<box_size):
		ind1=0
	    else:
		ind1=ind-box_size
	    if(ind>p.shape[0]-box_size-1):
		ind2=p.shape[0]-1
	    else:
		ind2=ind+box_size
	    if(mean<box_size):
		ind3=0
	    else:
		ind3=mean-box_size
	    if(mean>p.shape[1]-box_size-1):
		ind4=p.shape[1]-1
	    else:
		ind4=mean+box_size
	    new_img=p[ind1:ind2,ind3:ind4]
	    return (new_img)
	def arrow_angle(self,frame):
	    print('in')
	    cv2.imwrite('Images123/image1.jpg', frame)
	    frame=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
	    frame = frame
	    imgg= np.copy(frame)
	    imgg[(imgg[:,:,0]).all()>250 and (imgg[:,:,1]).all()<150]=255
	    imgg[imgg[:,:,0]<=250]=0
	    imgg[:,:,1]=imgg[:,:,0]
	    imgg[:,:,2]=imgg[:,:,0]
	    
	    kernel = np.ones((5,5),np.uint8)	
	    imgg = cv2.morphologyEx(imgg, cv2.MORPH_OPEN, kernel)
	    imgg = cv2.morphologyEx(imgg, cv2.MORPH_CLOSE, kernel)
	    cv2.imwrite('Images123/imgg.jpg', imgg)

	    imgg=self.crop_img(imgg)
	    lines=[]
	    self.copy_img=np.copy(imgg)#used to rotate 
	    imgg = cv2.cvtColor(imgg,cv2.COLOR_GRAY2RGB)
	    edges = cv2.Canny(imgg,1,250)
	    lines = cv2.HoughLines(edges,1,np.pi/180,90)
	    #print(lines)
	    if(type(lines) is np.ndarray):
		
		    for i in range(0,lines.shape[0]):
			for rho,theta in lines[i]:
			    a = np.cos(theta)
			    b = np.sin(theta)
			    x0 = a*rho
			    y0 = b*rho
			    x1 = int(x0 + 1000*(-b))
			    y1 = int(y0 + 1000*(a))
			    x2 = int(x0 - 1000*(-b))
			    y2 = int(y0 - 1000*(a))
			    cv2.line(imgg,(x1,y1),(x2,y2),(0,0,255),5)
		    dist = 80 #distance between two parallel lines
		    cv2.imwrite('Images123/hough.jpg', imgg)
		    bb=0
		    angle=(lines[0][0][1])
		    for i in range(0,len(lines)):
			for j in range(i+1, len(lines)):
			    if (np.abs(lines[i][0][0]-lines[j][0][0])>=dist-1 and np.abs(lines[i][0][1]-lines[j][0][1])<0.03):
				angle=(lines[j][0][1])
				bb=1
				break
			if bb==1:
			    break
		    if(angle>=np.pi/2):
			final_angle=angle-np.pi
		    else :
			final_angle=angle
		    #FINALLY ROTATE IMAGE
		    rot_img = self.rotateImage(self.copy_img,final_angle*180/np.pi)
		    rot_img= cv2.medianBlur(rot_img, 21)
		    count=[]
		    mind=0
		    c=0
		    summ=np.sum(rot_img,axis=1)
		    if(np.abs(summ[np.argmax(summ)+10]-summ[np.argmax(summ)+15])>3000):
			#print('up')
			orient=-final_angle*180/np.pi
			#print(orient)
		    else:
			#print('down')
			orient=(final_angle*180/np.pi)
			if(orient >0):
			    orient = orient+90
			else:
			    orient = orient-90
			#print(orient)
		    return (orient)

	    else:	
		    return(1000)



