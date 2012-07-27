#!/usr/bin/env python
import roslib
roslib.load_manifest('google_goggles')
import rospy
import os, sys, tempfile
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

import urllib, urllib2

class GoogleGogglesConnector(object):

    SERVER = "http://amp.google.com/"
    LEARN = "objreco/learn"
    MATCH = "objreco/match"

    @staticmethod
    def _prepare_request(url, img_path):
        length = os.path.getsize(img_path)
        img_data = open(img_path, "rb")
        request = urllib2.Request(url, data=img_data)
        request.add_header('Cache-Control', 'no-cache')
        request.add_header('Content-Length', '%d' % length)
        request.add_header('Content-Type', 'image/png')
        return request

    @staticmethod
    def _prepare_url(base, params):
        return base + "?" + urllib.urlencode(params)

    @staticmethod
    def learn(img_path, label):
        url = GoogleGogglesConnector._prepare_url( \
                GoogleGogglesConnector.SERVER + GoogleGogglesConnector.LEARN,
                {"label": label, "serverid": 1})
        request = GoogleGogglesConnector._prepare_request(url, img_path)
        res = urllib2.urlopen(request).read().strip()
        return res

    @staticmethod
    def match(img_path):
        url = GoogleGogglesConnector._prepare_url( \
                GoogleGogglesConnector.SERVER + GoogleGogglesConnector.MATCH,
                {"serverid": 1})
        request = GoogleGogglesConnector._prepare_request(url, img_path)
        res = urllib2.urlopen(request).read().strip()
        return res

class image_tester:

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
		self.last_test_time = None
		self.name = "img"
		self.folder = "../data/images"
		self.crop_size = None
		self.crop_offset = (0,0)
		self.test = test
		
		if len(argv) >= 1:
			self.name = argv[0]
			
		if len(argv) >= 2:
			self.interval = rospy.Duration(int(argv[1]))
		
		if len(argv) >= 3:
			self.crop_size = (int(argv[2]),int(argv[3]))
		
		if len(argv) >= 5:
			try:
				self.crop_offset = (int(argv[4]), int(argv[5]))
			except:
				if argv[4] == "center":
					self.crop_offset = ((2050 - self.crop_size[0])/2,(2448 - self.crop_size[1])/2)
					if len(argv) >= 6:
						self.crop_offset = (self.crop_offset[0] + int(argv[5]),self.crop_offset[1] + int(argv[6]))
		
		roslib.log_msg("Testing images at interval %d" % (self.folder,self.name,self.interval.to_sec()))
		if self.crop_size is not None:
			roslib.log_msg("Crop is %s with offset %s" % (self.crop_size,self.crop_offset))
		#rospy.init_node('image_saver', anonymous=True)

	def callback(self,data):
		if self.last_test_time is None or (data.header.stamp - self.last_test_time > self.interval):
			try:
				self.image = self.bridge.imgmsg_to_cv(data, "bgr8")
			except CvBridgeError, e:
				print e
			
			if self.crop_size is None:
				self.cropped_image = self.image
			else:
				self.cropped_image = self.image[self.crop_offset[0]:self.crop_offset[0]+self.crop_size[0],self.crop_offset[1]:self.crop_offset[1]+self.crop_size[1]]
			
			stamp = data.header.stamp.to_time()
			#filename = self.name + "_" + time.strftime("%Y_%m_%d_T%H_%M_%S",time.localtime(stamp))
			tmp = tempfile.TemporaryFile()
			filename = tmp.name + ".jpg"
			if self.test:
				print "got image of size (%d,%d)" % (self.image.cols,self.image.rows)
			else:
				print "savtesting ing image of size (%d,%d)" % (self.image.cols,self.image.rows)
				cv.SaveImage(filename,self.cropped_image)
				res = GoogleGogglesConnector.match(filename)
				#TODO: display
			self.last_test_time = data.header.stamp

		#cv.ShowImage("raw", self.image)
		cv.ShowImage("cropped", self.cropped_image)
		cv.WaitKey(3)



def main(argv):
	rospy.init_node('image_tester', anonymous=True)
	arg_start = 1
	test = False
	if len(argv) > 1 and argv[1] == "test":
		print "not saving"
		test = True
		arg_start = 2
	ic = image_tester(argv[arg_start:],test)
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv.DestroyAllWindows()

if __name__ == "__main__":
	main(sys.argv)
