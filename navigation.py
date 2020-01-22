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
from commands import Commands
from subscribers import Subscribers
from computations import Computations

#class for navigation functions
class Navigation():
	#initialize
	def __init__(self):
		#self.rate = rospy.Rate(10)
		self.subs=Subscribers()	
		self.commands=Commands()	
		self.pose = PoseStamped()
		self.comp=Computations()
		self.setpoint_publisher = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
	def stillActive(self):
		return self.subs.state.mode == 'OFFBOARD'
	def assign(self,t):							#function to assign values
		
		self.pose.pose.position.x = t[0]
	  	self.pose.pose.position.y = t[1]
		self.pose.pose.position.z = t[2]
		a = t[0] - self.subs.pose.pose.position.x
		b = t[1] - self.subs.pose.pose.position.y
		tanin = (atan2(b,a))
		quat = quaternion_from_euler(0,0,tanin)
		self.pose.pose.orientation.x = quat[0]
		self.pose.pose.orientation.y = quat[1]
		self.pose.pose.orientation.z = quat[2]
		self.pose.pose.orientation.w = quat[3]
		#self.pose.pose.orientation=self.subs.pose.pose.orientation	
	def waypoint(self,way_x,way_y,way_z):					#function to publish and travel to the next waypoint
		rate = rospy.Rate(10)		
		x=way_x
		y=way_y
		z=way_z
		wp_list_example=[(x,y,z)]
		
			
		
			
		while not len(wp_list_example) == 0:
			t = wp_list_example.pop()
			self.assign(t)
				
			print('going to', self.pose)
			while abs(self.comp.compute_distance(self.pose,self.subs.pose)) > 0.5:				
				if(self.stillActive()):
					self.setpoint_publisher.publish(self.pose)					
					rate.sleep()
				else:	
					print('exiting coz mode changed')					
					return	
				





			'''if(self.subs.state.mode =='AUTO.LOITER' or self.subs.state.mode =='STABILIZED'):
						
				self.commands.set_mode(self.subs.state.mode)
					
			else:
				if abs(self.comp.compute_distance(self.pose,self.subs.pose)) > 0.5:
										
					self.setpoint_publisher.publish(self.pose)
					rate.sleep()
				else:
						
					t = wp_list_example.pop()
					self.assign(t)

			'''
