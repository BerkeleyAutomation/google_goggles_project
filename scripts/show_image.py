#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('google_goggles')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import time
import rosbag

window_name = "image"

class image_shower:

	def __init__(self,argv,show):
		self.show = show
		
		self.image_sub_topic = "/prosilica/image_color";
		self.points_sub_topic = "/camera/depth_registered/points";
		
		cv.NamedWindow(window_name, 1)
		self.bridge = CvBridge()
		
		self.image_sub = rospy.Subscriber(self.image_sub_topic,Image,self.image_callback)
		
		self.image = None
		
		self.interval = rospy.Duration(3)
		self.last_image_time = None
	def image_callback(self,data):
		#print 'img'
		if self.last_image_time is None or (data.header.stamp - self.last_image_time > self.interval):
			stamp = data.header.stamp.to_time()
			self.image = data
			self.last_image_time = data.header.stamp
			print "got image  at %s  with size (%d,%d)" % (time.strftime("%Y_%m_%d_T%H_%M_%S",time.localtime(stamp)),self.image.width,self.image.height)
			self.image_pub.publish(self.image)
		if self.show:
			try:
				cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
				cv.ShowImage(window_name, cv_image)
				cv.WaitKey(3)
			except CvBridgeError, e:
				print e
		
	
	def points_callback(self,data):
		#print 'points'
		if self.last_points_time is None or (data.header.stamp - self.last_points_time > self.interval):
			stamp = data.header.stamp.to_time()
			self.points = data
			self.last_points_time = data.header.stamp
			print "got points at %s" % (time.strftime("%Y_%m_%d_T%H_%M_%S",time.localtime(stamp)))
			self.points_pub.publish(self.points)
		
		

def main(argv):
	print "starting node"
	rospy.init_node('data_saver', anonymous=True)
	print "node started"
	arg_start = 1
	show = False
	if len(argv) > 1 and argv[1] == "-show":
		show = True
		arg_start = 2
	ds = data_saver(argv[arg_start:],show)
	
	#try:
		#while not rospy.is_shutdown():
			#print "publishing..."
			#if ds.image is not None:
				##ds.bag.write(ds.image_sub_topic,ds.image.header.stamp,ds.image)
			#if ds.points is not None:
				##ds.bag.write(ds.points_sub_topic,ds.points.header.stamp,ds.points)
			#rospy.sleep(ds.interval)
	#finally:
		#ds.bag.close()
	
	rospy.spin()
	
	cv.DestroyAllWindows()

if __name__ == "__main__":
	main(sys.argv)
