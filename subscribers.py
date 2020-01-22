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


class Subscribers():

	def __init__(self):
		#initializing the class variables
    		self.pose = PoseStamped()
		self.glob_pos = NavSatFix()
	    	self.velocity = TwistStamped()
		self.state = State()		
		#self.bridge = CvBridge()
	
	    	# streaming of subscribers
	    	rospy.wait_for_service("mavros/set_stream_rate")
	    	set_stream_rate=rospy.ServiceProxy("mavros/set_stream_rate",StreamRate)
	    	set_stream_rate(StreamRateRequest.STREAM_POSITION, 50, True)
	    	set_stream_rate(StreamRateRequest.STREAM_EXTRA1, 50, True)
	        
		#camera 
		#self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.callback)

	    	# position
	    	rospy.Subscriber("/mavros/local_position/pose",PoseStamped,self.process_position)
	    
	    	# global position
		rospy.Subscriber("/mavros/global_position/global",NavSatFix,self.glob_posi)
		

		#mode 
	    	rospy.Subscriber("/mavros/state",State,self.process_state)
	    
	    	# velocity
	    	rospy.Subscriber("/mavros/local_position/velocity_local",TwistStamped,self.process_velocity)
	    
	def process_state(self,state):
	    	self.state = state

	def process_position(self,pose):	
		self.pose = pose
	
	def glob_posi(self,glob_pos): 	
		self.glob_pos = glob_pos  
	  
	def process_velocity(self,velocity):
		self.velocity = velocity

	#def callback(self,data):
		
		#self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	


