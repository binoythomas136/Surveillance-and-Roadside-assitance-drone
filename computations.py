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
from subscribers import Subscribers

#class for all computations
class Computations():
	def __init__(self):
		self.rate=rospy.Rate(10)
		self.subs=Subscribers()
		self.setpoint_publisher = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
		self.pose = PoseStamped()

	def stillActive(self):
		return (self.subs.state.mode == 'OFFBOARD')

	def compute_distance(self,pose_a,pose_b):					#compute distance between 2 local positions
	    	a_x = pose_a.pose.position.x
	   	a_y = pose_a.pose.position.y
	    	a_z = pose_a.pose.position.z
	    	b_x = pose_b.pose.position.x
	    	b_y = pose_b.pose.position.y
	    	b_z = pose_b.pose.position.z  
		return ((a_x - b_x) ** 2 + (a_y - b_y) ** 2 + (a_z - b_z) ** 2) ** 0.5 	


	def send_arb_waypoints(self):							#function to assiign arbitary no. of waypoints
		
		posx=self.subs.pose.pose.position.x
		posy=self.subs.pose.pose.position.y
		posz=self.subs.pose.pose.position.z		
		wp_list_example=[]
		for i in range(10):
			wp_list_example.append((posx,posy,posz))		
		pose = PoseStamped()
		for i in range(10):
			t = wp_list_example.pop()
			pose.pose.position.x = t[0]
			pose.pose.position.y = t[1]
			pose.pose.position.z = t[2]
			pose.pose.orientation=self.subs.pose.pose.orientation

	      		self.setpoint_publisher.publish(pose)
			self.rate.sleep()
		print('sent')

	def change_orientation(self,delta_orientation):					#change orientation of drone according to arrow
		
		q=[self.subs.pose.pose.orientation.x, self.subs.pose.pose.orientation.y, self.subs.pose.pose.orientation.z, self.subs.pose.pose.orientation.w]							#present quaternion values
		rpy=euler_from_quaternion(q)						#convert to euler			
		yaw=((rpy[2]*180)/3.142)						#convert yaw to degrees
		print('present yaw')			
		print(yaw)								
		new_yaw=yaw+delta_orientation						#get new yaw from the change in degrees
		if new_yaw > 180:	
			new_yaw=new_yaw -360
		if new_yaw < -180:
			new_yaw =new_yaw+360
		print('final yaw')
		print(new_yaw)
		new_yaw_rad=(new_yaw*3.142/180)						#convert to radians
		quat=quaternion_from_euler(0,0,new_yaw_rad)				#convert to quaternions

		pose = PoseStamped()							#initialize the final pose values
		pose.pose.position.x = self.subs.pose.pose.position.x
		pose.pose.position.y = self.subs.pose.pose.position.y
		pose.pose.position.z = self.subs.pose.pose.position.z
		pose.pose.orientation.x=quat[0]
		pose.pose.orientation.y=quat[1]
		pose.pose.orientation.z=quat[2]
		pose.pose.orientation.w=quat[3]



		while((abs(new_yaw-yaw))>4):

			'''pose.pose.position.x = self.subs.pose.pose.position.x
			pose.pose.position.y = self.subs.pose.pose.position.y
			pose.pose.position.z = self.subs.pose.pose.position.z
			pose.pose.orientation.x=quat[0]
			pose.pose.orientation.y=quat[1]
			pose.pose.orientation.z=quat[2]
			pose.pose.orientation.w=quat[3]	'''
			self.setpoint_publisher.publish(pose)				#publish the changed orientation
		  	#self.rate.sleep()

	def road_next_waypoint(self,delta_orientation):
		delta_orientation_rad = radians(delta_orientation)			#change in road orientation
		print(delta_orientation_rad)
			
		length = 3								#distance by which the drone will move
		next_diff_x = -length*sin(delta_orientation_rad)			
		next_diff_y = length*cos(delta_orientation_rad)				
		next_x=self.subs.pose.pose.position.x+next_diff_x			#next waypoints x value	
		next_y=self.subs.pose.pose.position.y+next_diff_y			#next waypoints y value
		print(self.subs.pose.pose.position.x,self.subs.pose.pose.position.y)
		print(next_x,next_y)
		q=[	self.subs.pose.pose.orientation.x, 
			self.subs.pose.pose.orientation.y, 
			self.subs.pose.pose.orientation.z, 
			self.subs.pose.pose.orientation.w	]			#present orientation quaternions
		rpy=euler_from_quaternion(q)						#euler values
		yaw=((rpy[2]*180)/3.142)						#present yaw
		final_yaw=yaw+delta_orientation						#final yaw
		final_yaw_rad=final_yaw*(3.142/180)					#final yaw in radians
		
		quat=quaternion_from_euler(0,0,final_yaw_rad)				#quaternion values of final yaw
		#pose values to be published		
		self.pose.pose.position.x=next_x					
		self.pose.pose.position.y=next_y
		self.pose.pose.position.y=self.subs.pose.pose.position.z
		self.pose.pose.orientation.x=quat[0]
		self.pose.pose.orientation.y=quat[1]
		self.pose.pose.orientation.z=quat[2]
		self.pose.pose.orientation.w=quat[3]
		#loop to check if the drone has reached the waypoint
		while( abs(self.compute_distance(self.pose,self.subs.pose)) > 0.5):
			if(self.stillActive()):
				self.setpoint_publisher.publish(self.pose)



		

