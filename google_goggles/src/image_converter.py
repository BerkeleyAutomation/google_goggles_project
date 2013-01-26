#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('google_goggles')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

	def __init__(self):
		#self.image_pub = rospy.Publisher("image_topic_2",Image)

		cv.NamedWindow("Raw image", 1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/wide_stereo/left/image_color",Image,self.callback)
		self.image = None
		rospy.init_node('image_converter', anonymous=True)

	def callback(self,data):
		try:
			self.image = self.bridge.imgmsg_to_cv(data, "bgr8")
		except CvBridgeError, e:
			print e

		cv.ShowImage("Raw image", self.image)
		cv.WaitKey(3)
		
	def kill(self):
		self.image_sub.unregister()
		cv.DestroyWindow("Raw image")