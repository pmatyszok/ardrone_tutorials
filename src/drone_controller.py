#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
import time

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from sensor_msgs.msg import Image    	 # for receiving the video feed
from ardrone_autonomy.srv import FlightAnim
# An enumeration of Drone Statuses
from drone_status import DroneStatus
from time import sleep
#for QImage class

from PySide import QtCore, QtGui

import cv
from cv_bridge import CvBridge, CvBridgeError
# Some Constants
COMMAND_PERIOD = 100 #ms


class BasicDroneController(object):
	
	def __init__(self):
		# Holds the current drone status
		self.status = -1

		self.rotating = 0
		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)
		
		rospy.wait_for_service('/ardrone/setflightanimation');
		#self.animSrv = rospy.Publisher('/ardrone/setflightanimation',Empty)
		
		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist)

		# Setup regular publishing of control packets
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

		# Land the drone if we are shutting down
		rospy.on_shutdown(self.SendLand)
		
		self.doTakeImage = False
		self.imageSource = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)
		self.inspectionCounter = 0
		self.image = None
		self.bridge = CvBridge()
	
	def ReceiveImage(self, data):
		if self.doTakeImage == True :
			self.doTakeImage = False
			self.image = self.bridge.imgmsg_to_cv(data, "bgr8")
			fileName = "/home/pawel/inspections/inspection" + str(self.inspectionCounter) + "_" + time.strftime("%m%d-%H%M%S") + ".png"
			print fileName
			cv.SaveImage(fileName, self.image)
			

	def ReceiveNavdata(self,navdata):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment	
		self.status = navdata.state

	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.status == DroneStatus.Landed):
			self.pubTakeoff.publish(Empty())

	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())


	def StartRotating(self):
		try:
			self.inspectionCounter = self.inspectionCounter + 1
			self.doTakeImage = True
			sleep(0.05)
		except rospy.ServiceException, e:
			print "taking print-screen failed: %s"%e

			
	def SetCommand(self,roll=0,pitch=0,yaw_left=0,yaw_right=0,z_velocity_up=0, z_velocity_down=0):
		# Called by the main program to set the current command
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		z_velocity = 0;
		if z_velocity_up != 1.0:
			z_velocity = -(z_velocity_up)/2 + 0.5
		elif z_velocity_down != 1.0:
			z_velocity = -(-(z_velocity_down)/2 + 0.5)
		self.command.linear.z  = z_velocity
		print z_velocity
		yaw_velocity = 0
		if yaw_left != 0:
			yaw_velocity = 1.0
		elif yaw_right != 0:
			yaw_velocity = -1.0
		print yaw_velocity
		self.command.angular.z = yaw_velocity

	def SendCommand(self,event):
		# The previously set command is then sent out periodically if the drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)

