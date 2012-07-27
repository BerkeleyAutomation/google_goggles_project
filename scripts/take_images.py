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
import time

class image_saver:

	def __init__(self,argv,test):
		#self.image_pub = rospy.Publisher("image_topic_2",Image)

		#cv.NamedWindow("raw", 1)
		cv.NamedWindow("cropped", 1)
		self.bridge = CvBridge()
		#self.image_sub = rospy.Subscriber("/wide_stereo/left/image_color",Image,self.callback)
		self.image_sub = rospy.Subscriber("/prosilica/image_color",Image,self.callback)
		self.image = None
		self.cropped_image = None
		self.interval = rospy.Duration(1)
		self.last_save_time = None
		self.name = "img"
		self.folder = "../data/images"
		self.save_size = None
		self.save_offset = (0,0)
		self.test = test
		
		if len(argv) >= 1:
			self.name = argv[0]
			
		if len(argv) >= 2:
			self.interval = rospy.Duration(int(argv[1]))
		
		if len(argv) >= 3:
			self.save_size = (int(argv[2]),int(argv[3]))
		
		if len(argv) >= 5:
			try:
				self.save_offset = (int(argv[4]), int(argv[5]))
			except:
				if argv[4] == "center":
					self.save_offset = ((2050 - self.save_size[0])/2,(2448 - self.save_size[1])/2)
					if len(argv) >= 6:
						self.save_offset = (self.save_offset[0] + int(argv[5]),self.save_offset[1] + int(argv[6]))
		
		print "Saving images to %s with prefix %s at interval %d" % (self.folder,self.name,self.interval.to_sec())
		if self.save_size is not None:
			print "Crop is %s with offset %s" % (self.save_size,self.save_offset)
		#rospy.init_node('image_saver', anonymous=True)

	def callback(self,data):
		if self.last_save_time is None or (data.header.stamp - self.last_save_time > self.interval):
			try:
				self.image = self.bridge.imgmsg_to_cv(data, "bgr8")
			except CvBridgeError, e:
				print e
			
			if self.save_size is None:
				self.cropped_image = self.image
			else:
				self.cropped_image = self.image[self.save_offset[0]:self.save_offset[0]+self.save_size[0],self.save_offset[1]:self.save_offset[1]+self.save_size[1]]
			
			stamp = data.header.stamp.to_time()
			filename = self.name + "_" + time.strftime("%Y_%m_%d_T%H_%M_%S",time.localtime(stamp))
			if self.test:
				print "got image %s of size (%d,%d)" % (filename,self.image.cols,self.image.rows)
			else:
				print "saving image %s of size (%d,%d)" % (filename,self.image.cols,self.image.rows)
				cv.SaveImage(self.folder + "/" + filename + "_raw.jpg",self.image);
				cv.SaveImage(self.folder + "/" + filename + "_crop.jpg",self.cropped_image);
			self.last_save_time = data.header.stamp

		#cv.ShowImage("raw", self.image)
		cv.ShowImage("cropped", self.cropped_image)
		cv.WaitKey(3)

def main(argv):
	rospy.init_node('image_saver', anonymous=True)
	arg_start = 1
	test = False
	if len(argv) > 1 and argv[1] == "test":
		print "not saving"
		test = True
		arg_start = 2
	ic = image_saver(argv[arg_start:],test)
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv.DestroyAllWindows()

if __name__ == "__main__":
	main(sys.argv)
