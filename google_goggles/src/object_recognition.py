import roslib
roslib.load_manifest("google_goggles")

import sys, os, time, tempfile, os.path
from optparse import OptionParser
from collections import defaultdict

from google_goggles_connector import GoogleGoggles

from math import *
import numpy
import rospy
import rosbag

import cv
from google_goggles_msgs.msg import ObjectReferenceData
from geometry_msgs.msg import *
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge, CvBridgeError
import tf
import tf.transformations as tft

TIME_FORMAT = "%Y-%m-%d-T%H-%M-%S"

RESULTS_STRING_FMT = '{test_type} {success} {input_label} {output_label} {stamp} {filename}'
def format_results(test_type,success,input_label,output_label,stamp,filename):
	return RESULTS_STRING_FMT.format(
		test_type=test_type,
		success=success,
		input_label=input_label,
		output_label=output_label,
		stamp=time.strftime(TIME_FORMAT,stamp),
		filename=filename)

class ObjectRecognition:
	def __init__(self,options,image_topic='camera',image_label_topic='image_label'):
		self.options = options
		if self.options.window:
			cv.NamedWindow("cropped", 1)
		self.bridge = CvBridge()
		
		self.image = None
		self.cropped_image = None
		self.interval = rospy.Duration(options.image_interval)
		self.last_test_time = None
		self.crop_size = None
		self.crop_offset = (0,0)
		self.test = options.test
		self.total_tests = 0
		
		self.total_successes = 0
		self.total_tests = 0
		self.label_successes = defaultdict(int)
		self.label_total = defaultdict(int)
		
		self.results_file = None
		if self.options.save_results:
			results_file_prefix = 'results'
			if self.options.save_results_to:
				results_file_prefix = self.options.save_results_to
				
			results_file_name = results_file_prefix + '_' + time.strftime(TIME_FORMAT,time.localtime()) + '.txt'
			try:
				os.makedirs(os.path.dirname(results_file_name))
				self.results_file = open(results_file_name,'w')
			except os.error, e:
				print e
		
		if options.crop_size is not None:
			sz = options.crop_size.split('x')
			self.crop_size = (int(sz[0]),int(sz[1]))
		
		if options.crop_center:
			self.crop_offset = ((2050 - self.crop_size[0])/2,(2448 - self.crop_size[1])/2)
		
		if options.crop_offset is not None:
			offset = options.crop_offset.split("x")
			self.crop_offset = (self.crop_offset[0] + int(offset[0]),self.crop_offset[1] + int(offset[1]))
		
		rospy.loginfo("Testing images at interval {0}".format(self.interval.to_sec()))
		if self.crop_size is not None:
			rospy.loginfo("Crop is {0} with offset {1}".format(self.crop_size,self.crop_offset))
		
		self.image_label_pub = rospy.Publisher(image_label_topic,String,latch=True)
		self.image_sub_topic = image_topic
		self.image_sub = rospy.Subscriber(self.image_sub_topic,Image,self.callback)

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
			
			time_str = time.strftime("%H:%M:%S",time.localtime(data.header.stamp.to_time()))
			
			if self.crop_size is None:
				self.cropped_image = self.image
			else:
				self.cropped_image = self.image[self.crop_offset[0]:self.crop_offset[0]+self.crop_size[0],self.crop_offset[1]:self.crop_offset[1]+self.crop_size[1]]
		
		#cv.ShowImage("raw", self.image)
		if self.options.window:
			cv.ShowImage("cropped", self.cropped_image)
			cv.WaitKey(3)
		
		if process:
			stamp = time.localtime(data.header.stamp.to_time())
			stamp_string = time.strftime(TIME_FORMAT,stamp)
			#filename = self.name + "_" + stamp_string
			tmp = tempfile.NamedTemporaryFile(dir='/tmp')
			filename = tmp.name + ".jpg"
			rospy.logdebug("got image of size ({0},{1}) with stamp {2}".format(self.image.cols,self.image.rows,time_str))
			if not self.test:
				cv.SaveImage(filename,self.cropped_image)
				save_file_name = ''
				if self.options.save_image:
					save_file_name = self.save_image + "_" + stamp_string + ".jpg"
					try:
						os.makedirs(os.path.dirname(save_file_name))
						cv.SaveImage(save_file_name,self.cropped_image)
					except os.error, e:
						print e
				
				results_string = None
				if self.options.learn_image and not self.options.no_google:
					print "learning...",
					if self.options.fake_google:
						res = {'status':'SUCCESS', 'image_label':self.options.learn_image}
					else:
						res = GoogleGoggles.learn(filename,self.options.learn_image)
					self.total_tests += 1
					if res['status'] == 'SUCCESS':
						print 'success! label is {0}'.format(res['image_label'])
						results_string = format_results('LEARN','SUCCESS',self.options.learn_image,res['image_label'],stamp,save_file_name)
					else:
						print 'failure :('
						results_string = format_results('LEARN','FAILURE',self.options.learn_image,'~',stamp,save_file_name)
				elif not self.options.no_google:
					print "testing...",
					if self.options.fake_google:
						res = {'status':'SUCCESS', 'image_label':self.options.fake_google}
					else:
						res = GoogleGoggles.match(filename)
					#print res
					res_label = res['image_label']
					if res_label:
						self.label_total[res_label] += 1
					else:
						self.label_total['FAILURE'] += 1
					self.total_tests += 1
					
					if self.options.label:
						test_label = self.options.label
						success = res_label == test_label
						if success:
							print 'successful recognition of {0}!'.format(test_label)
							self.total_successes += 1
							#self.label_successes[test_label] = self.label_successes[test_label] + 1
							results_string = format_results('TEST','SUCCESS',self.options.label,res_label,stamp,save_file_name)
						elif res_label:
							print 'failure, recognized as {0}'.format(res_label)
							results_string = format_results('TEST','FAILURE',self.options.label,res_label,stamp,save_file_name)
						else:
							print 'failure :('
							results_string = format_results('TEST','FAILURE',self.options.label,'~',stamp,save_file_name)
					else:
						if res['status'] == 'SUCCESS':
							label = res['image_label']
							print 'success! label is {0}'.format(label)
							self.total_successes += 1
							
							results_string = format_results('TEST','SUCCESS','~',label,stamp,save_file_name)
							
							self.image_label_pub.publish(str(label))
						else:
							print 'failure :('
							results_string = format_results('TEST','FAILURE','~','~',stamp,save_file_name)
					if False:
						print 'successes: {0}/{1}'.format(self.total_successes,self.total_tests)
						for label in self.label_total.keys():
							print "  {0}: {1}".format(label,self.label_total[label])
				if results_string and self.results_file:
					results_file.write(results_string + '\n')
					
			tmp.close()
			self.last_test_time = data.header.stamp
		
		if self.options.max and self.total_tests >= self.options.max:
			print "max number of tests reached"
