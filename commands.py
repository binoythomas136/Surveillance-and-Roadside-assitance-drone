#!/usr/bin/env python
#import statements
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
import unittest


#class for commands for the drone
class Commands():
	def __init__(self):
		self.subs=Subscribers()		
		self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)		
		self.setpoint_publisher = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
	
	'''def set_mode(self,mode):							#function to set mode of the drone

		rospy.wait_for_service('/mavros/set_mode')				
	    	try:
			command_service = rospy.ServiceProxy('/mavros/set_mode',SetMode)#ROS service
	    		#mode1=mode
			#num = 0 if mode1 == mode else 1
			num=0
	    		resp=command_service(num,mode)	
			print ("Mode Changed :"+ str(resp.mode_sent))
		except rospy.ServiceException, e:
	    		print "Failed SetMode: %s" % e
	'''	

        def set_mode(self, mode):
		rate = rospy.Rate(5)
	    	resp=self.set_mode_srv(0,mode)	
		print ("Mode Changed :"+ str(resp.mode_sent))
                while (self.subs.state.mode != mode):
			self.setpoint_publisher.publish(self.subs.pose)
                        print("New mode: "+str(self.subs.state.mode))
                        rate.sleep()
                print("Got it: "+str(self.subs.state.mode))

        '''
	def set_mode(self, mode, timeout):
		"""mode: PX4 mode string, timeout(int): seconds"""
		print("setting FCU mode: {0}".format(mode))
		old_mode = self.subs.state.mode
		loop_freq = 1  # Hz
		rate = rospy.Rate(loop_freq)
		mode_set = False
		
		for i in xrange(timeout * loop_freq):
		    if self.subs.state.mode == mode:
		        mode_set = True
		        print("set mode success | seconds: {0} of {1}".format(
		            i / loop_freq, timeout))
		        break
		    else:
		        try:
		            res = self.set_mode_srv(0, mode)  # 0 is custom mode
		            if not res.mode_sent:
		                print("failed to send mode command")
		        except rospy.ServiceException as e:
		            print(e)

		    try:
		        rate.sleep()
		    except rospy.ROSException as e:
		        self.fail(e)

		#assertTrue(mode_set, ("failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2}".format(mode, old_mode, timeout)))
        '''
        
	def arm(self):									#function for arming the drone
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			arm_service = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)#ROS service
			resp=arm_service(True)
			print ("Arming :"+ str(resp.success))
		except rospy.ServiceException, e:
	    		print "Failed Arming: %s" % e
	def disarm(self):								#function for disarming the drone
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			arm_service = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
			resp=arm_service(False)
			print ("Arming :"+ str(resp.success))
		except rospy.ServiceException, e:
	    		print "Failed Disarming: %s" % e

	def land(self):
		self.set_mode('AUTO.LAND')						#autopilot land					
		'''
		land_lat=self.subs.glob_pos.latitude					#global latitude
		land_lon=self.subs.glob_pos.longitude					#global longitude
		land_alt=self.subs.glob_pos.altitude					#global altitude
	 	
		rospy.wait_for_service('/mavros/cmd/land')				
		try:
		    	land_service = rospy.ServiceProxy('/mavros/cmd/land',CommandTOL)#service
		 	land_service(0,0,land_lat,land_lon,2)
		except rospy.ServiceException, e:
	    		print "Failed landing: %s" % e
		rate = rospy.Rate(10)
		
		
		
		print('reached an altiude of 2 m')
		for i in range(5):
			rate.sleep()
		#slow landing after 2 m
		prev_vel = self.subs.velocity.twist.linear.z
		curr_vel = self.subs.velocity.twist.linear.z
		print((prev_vel,curr_vel))
		while not (abs(prev_vel) < 0.05 and abs(curr_vel) < 0.05):
			print("Waiting to Land")
			print((prev_vel,curr_vel))
			curr_vel = self.subs.velocity.twist.linear.z
			prev_vel = curr_vel
			rate.sleep()
		'''
