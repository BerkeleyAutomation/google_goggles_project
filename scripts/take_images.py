#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('google_goggles')
import sys, os, time, tempfile, os.path
from optparse import OptionParser
import urllib, urllib2, json
from collections import defaultdict

from math import *
import numpy
import rospy
import rosbag

import cv
from geometry_msgs.msg import *
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import tf
import tf.transformations as tft

TIME_FORMAT = "%Y-%m-%d-T%H-%M-%S"

class image_saver:

	def __init__(self,options):
		self.options = options
		cv.NamedWindow("cropped", 1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/prosilica/image_rect_color",Image,self.callback)
		self.image = None
		self.cropped_image = None
		self.interval = rospy.Duration(options.interval)
		self.first_test_time = None
		self.last_test_time = None
		self.crop_size = None
		self.crop_offset = (0,0)
		self.test = options.test
		self.total_tests = 0
		
		self.save_file_dir = os.path.join(options.dir,options.object)
		try:
			os.makedirs(self.save_file_dir)
		except os.error, e:
			pass
		
		self.save_file_prefix = '_'.join([options.object,options.pose,options.bg,options.lighting,options.view])
		
		rospy.loginfo('Saving to dir {0} with prefix {1}'.format(self.save_file_dir,self.save_file_prefix))
		
		if options.crop_size is not None:
			sz = options.crop_size.split('x')
			self.crop_size = (int(sz[0]),int(sz[1]))
		
		if options.crop_center:
			self.crop_offset = ((2050 - self.crop_size[0])/2,(2448 - self.crop_size[1])/2)
		
		if options.crop_offset is not None:
			offset = options.crop_offset.split("x")
			self.crop_offset = (self.crop_offset[0] + int(offset[0]),self.crop_offset[1] + int(offset[1]))
		
		if not self.test:
			rospy.loginfo("Saving images at interval {0}".format(self.interval.to_sec()))
		else:
			rospy.loginfo("TEST at interval {0}".format(self.interval.to_sec()))
		if self.crop_size is not None:
			rospy.loginfo("Crop is {0} with offset {1}".format(self.crop_size,self.crop_offset))

	def callback(self,data):
		if self.options.max and self.total_tests >= self.options.max:
			cv.WaitKey(3)
			return
		process = self.last_test_time is None or (data.header.stamp - self.last_test_time > self.interval)
		if process:
			try:
				self.image = self.bridge.imgmsg_to_cv(data, "bgr8")
			except CvBridgeError, e:
				print e
			
			if self.crop_size is None:
				self.cropped_image = self.image
			else:
				self.cropped_image = self.image[self.crop_offset[0]:self.crop_offset[0]+self.crop_size[0],self.crop_offset[1]:self.crop_offset[1]+self.crop_size[1]]
		
		#cv.ShowImage("raw", self.image)
		cv.ShowImage("cropped", self.cropped_image)
		cv.WaitKey(3)
		
		if process:
			stamp = time.localtime(data.header.stamp.to_time())
			stamp_string = time.strftime(TIME_FORMAT,stamp)
			save_file_name = os.path.join(self.save_file_dir,self.save_file_prefix + '_' + stamp_string + '.jpg')
			if self.test:
				rospy.loginfo("got image of size ({0},{1}) NOT SAVING to:\n  {2}".format(self.image.cols,self.image.rows,save_file_name))
			else:
				rospy.loginfo("got image of size ({0},{1}), saving to:\n  {2}".format(self.image.cols,self.image.rows,save_file_name))
				
				cv.SaveImage(save_file_name,self.cropped_image)
				self.total_tests += 1
				
					
			self.last_test_time = data.header.stamp
			if not self.first_test_time:
				self.first_test_time = data.header.stamp
		
			if self.options.max and self.total_tests >= self.options.max:
				print "max number of tests reached"
				rospy.signal_shutdown("max number of tests reached")
			elif self.options.max_time and (data.header.stamp - self.first_test_time).to_sec() > self.options.max_time:
				print "max testing time reached"
				rospy.signal_shutdown("max testing time reached")
  

def main(argv):
	rospy.init_node('image_saver', anonymous=True)
	parser = OptionParser('%prog [OPTIONS] [OBJECT]')
	parser.add_option("--test",dest="test",action="store_true",default=False)
	
	parser.add_option('-d','--dir',default='')
	
	parser.add_option('-o','--object')
	parser.add_option('-p','--pose',default='NO-POSE')
	parser.add_option('-b','--background','--bg',dest='bg',default='NO-BG')
	parser.add_option('-l','--lighting',default='NO-LTNG')
	parser.add_option('-v','--view',default='NO-VIEW')
	
	parser.add_option("--max",type='int',default=0)
	parser.add_option("--single",action="store_const",dest="max",const=1)
	parser.add_option("--max-time",type='float')
	
	parser.add_option("-i","--interval",type="float",default=4.5)
	
	parser.add_option("--size",dest="crop_size",help="image crop size WxH")
	
	parser.add_option("--center",dest="crop_center",action="store_true",default=False)
	parser.add_option("--offset",dest="crop_offset")
	
	(options, args) = parser.parse_args()
	
	if not options.object:
		if args:
			options.object = args[0]
			del args[0]
		else:
			parser.error('No object given!')
		
	ic = image_saver(options)
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv.DestroyAllWindows()

if __name__ == "__main__":
	main(sys.argv)
