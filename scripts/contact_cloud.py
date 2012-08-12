#!/usr/bin/env python

import roslib
roslib.load_manifest("google_goggles")

import sys, os, time, tempfile
from optparse import OptionParser
import urllib, urllib2, json
from collections import defaultdict

from math import *
import numpy
import rospy

import cv
from geometry_msgs.msg import *
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import tf
import tf.transformations as tft

from brett2.PR2 import PR2
import openravepy

from actionlib import SimpleActionClient, SimpleGoalState
from arm_navigation_msgs.msg import *

from pr2_simple_motions_srvs.srv import *
from pr2_common_action_msgs.msg import *
from pr2_controllers_msgs.msg import *
from std_srvs.srv import Empty
import actionlib

def parseResponse(res):
	return json.loads(res.replace("status","'status'").replace("image_label","'image_label'").replace("match_score","'match_score'").replace("image_id","'image_id'").replace("'",'"'))

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
        return parseResponse(res)

    @staticmethod
    def match(img_path):
        url = GoogleGogglesConnector._prepare_url( \
                GoogleGogglesConnector.SERVER + GoogleGogglesConnector.MATCH,
                {"serverid": 1})
        request = GoogleGogglesConnector._prepare_request(url, img_path)
        print "requesting..."
        #print request.header_items()
        res = urllib2.urlopen(request).read().strip()
        return parseResponse(res)

class image_tester:
	def __init__(self,options):
		self.options = options
		cv.NamedWindow("cropped", 1)
		self.bridge = CvBridge()
		#self.image_sub = rospy.Subscriber("/wide_stereo/left/image_color",Image,self.callback)
		self.image_sub = rospy.Subscriber("/prosilica/image_color",Image,self.callback)
		self.image = None
		self.cropped_image = None
		self.interval = rospy.Duration(options.interval)
		self.last_test_time = None
		self.crop_size = None
		self.crop_offset = (0,0)
		self.test = options.test
		self.total_tests = 0
		
		self.total_successes = 0
		self.total_tests = 0
		self.label_successes = defaultdict(int)
		self.label_total = defaultdict(int)
		
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
		cv.ShowImage("cropped", self.cropped_image)
		cv.WaitKey(3)
		
		if process:
			stamp = data.header.stamp.to_time()
			#filename = self.name + "_" + time.strftime("%Y_%m_%d_T%H_%M_%S",time.localtime(stamp))
			tmp = tempfile.NamedTemporaryFile(dir='/tmp')
			filename = tmp.name + ".jpg"
			rospy.loginfo("got image of size ({0},{1}) with stamp {2}".format(self.image.cols,self.image.rows,time_str))
			if not self.test:
				cv.SaveImage(filename,self.cropped_image)
				if self.options.learn_image:
					print "learning..."
					res = GoogleGogglesConnector.learn(filename,self.options.learn_image)
					self.total_tests += 1
					if res['status'] == 'SUCCESS':
						print 'success! label is {0}'.format(res['image_label'])
					else:
						print 'failure :('
				else:
					print "testing..."
					res = GoogleGogglesConnector.match(filename)
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
						elif res_label:
							print 'failure, recognized as {0}'.format(res_label)
						else:
							print 'failure :('
					else:
						if res['status'] == 'SUCCESS':
							print 'success! label is {0}'.format(res['image_label'])
							self.total_successes += 1
						else:
							print 'failure :('
					print 'successes: {0}/{1}'.format(self.total_successes,self.total_tests)
					for label in self.label_total.keys():
						print "  {0}: {1}".format(label,self.label_total[label])
					
			tmp.close()
			self.last_test_time = data.header.stamp
		
		if self.options.max and self.total_tests >= self.options.max:
			print "max number of tests reached"

class Grasper:
	pr2 = None
	
	def __init__(self):
		self.move_arm = SimpleActionClient('move_right_arm', MoveArmAction)
		
		self.table_height = None

	def grasp_pose_callback(self,msg):
		print "called", self.table_height
		if not self.table_height: return
		base_frame = 'base_footprint'
		
		rospy.loginfo("opening gripper")
		self.pr2.rgrip.open()
		rospy.sleep(1.5)
		
		(tf_trans,tf_rot) = self.pr2.tf_listener.lookupTransform(msg.header.frame_id,base_frame,rospy.Time(0))
		msg_tf = numpy.mat(numpy.dot(tft.translation_matrix(tf_trans),tft.quaternion_matrix(tf_rot)))
		
		rospy.loginfo("moving arm")
		q = numpy.array([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
		p = numpy.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
		rot = numpy.mat(tft.quaternion_matrix(q))
		trans = numpy.mat(tft.translation_matrix(p))
		pose = msg_tf * trans * rot
		
		pose_x = pose[0,3]
		pose_y = pose[1,3]
		pose_z = pose[2,3]
		
		x_axis = (pose * numpy.matrix([1,0,0,1]).transpose())[0:3]
		
		init_dist = 0.1
		#first_pose = pose * tft.translation_matrix(-init_dist * x_axis.transpose())
		first_pose = pose * tft.translation_matrix(numpy.array([0,0,0.3]))
		
		for body in self.pr2.env.GetBodies():
			if body.GetName() == 'table': break
		else:
			rospy.loginfo('adding table to openrave env')
			body = openravepy.RaveCreateKinBody(self.pr2.env,'')
			body.SetName('table')
			body.InitFromBoxes(numpy.array([[pose_x,pose_y,self.table_height/2,0.25,1,self.table_height/2]]),False)
			self.pr2.env.AddKinBody(body)
		
		
		qt,pt = (tft.quaternion_from_matrix(first_pose),tft.translation_from_matrix(first_pose)) 
		#pr2.rarm.set_cart_target(qt,pt,msg.header.frame_id) #quat, xyz, ref_frame
		#joints = pr2.rarm.cart_to_joint(numpy.array(first_pose,base_frame,'r_gripper_tool_frame')
		print "first_pose:"
		print numpy.array(first_pose)
		
		filteropts = openravepy.openravepy_int.IkFilterOptions.CheckEnvCollisions
		filteropts = filteropts | openravepy.openravepy_int.IkFilterOptions.IgnoreSelfCollisions
		
		self.pr2.rarm.goto_pose_matrix(numpy.array(first_pose),base_frame,'r_gripper_tool_frame',filteroptions=filteropts); rospy.sleep(10)
		
		print "second pose:"
		print numpy.array(pose)
		#pr2.rarm.set_cart_target(tft.quaternion_from_matrix(pose),tft.translation_from_matrix(pose),msg.header.frame_id)
		self.pr2.rarm.goto_pose_matrix(numpy.array(pose),base_frame,'r_gripper_tool_frame'); rospy.sleep(5)
		
		rospy.loginfo("closing gripper")
		#pr2.rgrip.close(); rospy.sleep(3)
		
		rospy.loginfo("reseting")
		self.pr2.rarm.goto_posture('side')
		self.pr2.rgrip.open()

	def table_height_callback(self,msg):
		if not self.table_height: rospy.loginfo("got table height: %f" % msg.data)
		self.table_height = msg.data

if __name__ == "__main__":
	parser = OptionParser()
	parser.add_option("--test",dest="test",action="store_true",default=False)
	
	parser.set_defaults(ros=False)
	parser.add_option("--ros",dest="ros",action="store_true")
	parser.add_option("--no-ros",dest="ros",action="store_false")
	
	
	parser.add_option("-t","--test-files", action="store_true",default=False, help="use files")
	
	parser.add_option("--label", help='label to test against')
	
	parser.add_option("-l", "--learn",action="append",default=[],
					help="learn image FILE",metavar="FILE")
	parser.add_option("--learn-image",help='learn image from camera',metavar="LABEL")
					
	
	parser.add_option("--max",type='int',default=0)
	parser.add_option("--single",action="store_const",dest="max",const=1)
	
	parser.add_option("-i","--interval",type="float",default=4.5)
	
	parser.add_option('-s',"--size",dest="crop_size",help="image crop size WxH");
	
	parser.add_option("-c","--center",dest="crop_center",action="store_true",default=False);
	parser.add_option("-o","--offset",dest="crop_offset")
	
	parser.add_option("-g","--grasp",action="store_true",default=False, help="Run grasping");
	
	(options, args) = parser.parse_args()
	
	if options.learn_image:
		ans = raw_input('Are you sure you want to learn images with label {0}? '.format(options.learn_image))
		if ans != 'y' and ans != 'Y':
			exit(1)
		pass #options.max=1
	
	print options
	print args
	
	learnfiles = options.learn
	testfiles = []
	
	if options.test_files:
		if not args:
			parser.error("No files given for testing!")
		testfiles = args
	elif options.learn:
		learnfiles = learnfiles + args
	
	for (idx,learnfile) in enumerate(learnfiles):
		if options.label:
			label = options.label
		else:
			label = learnfile.split("_")[0]
		print "learning {0} from file {1} ({2}/{3})".format(label,learnfile,idx+1,len(learnfiles))
		res = GoogleGogglesConnector.learn(learnfile,label)
		print res
		if idx != len(learnfiles)-1: time.sleep(5)
	
	if options.test_files:
		total_successes = 0
		total_tests = 0
		label_successes = defaultdict(int)
		label_total = defaultdict(int)
		for (idx,testfile) in enumerate(testfiles):
			label = (testfile.split("/")[-1]).split("_")[0]
			print "testing file {0} ({1}/{2})".format(testfile,idx+1,len(testfiles))
			res = GoogleGogglesConnector.match(testfile)
			success = res['image_label'] == label
			if success:
				print 'success!'
				total_successes += 1
				label_successes[label] = label_successes[label] + 1
			else:
				print 'failure :('
			label_total[label] = label_total[label] + 1
			total_tests += 1
			if idx != len(testfiles)-1: time.sleep(5)
		print 'successes: {0}/{1}'.format(total_successes,total_tests)
		for label in label_successes.keys():
			print "  {0}: {1}/{2}".format(label,label_successes[label],label_total[label])
	elif options.ros:
		rospy.init_node('image_tester', anonymous=True)
		image_tester(options)
		
		rospy.loginfo('starting pr2')
		
		if options.grasp:
			grasper = Grasper()
			#grasper.pr2 = PR2()
			rospy.Subscriber("/google_goggles/grasp_pose", PoseStamped, grasper.grasp_pose_callback)
			rospy.Subscriber("/google_goggles/table_height", Float32, grasper.table_height_callback)
		
		rospy.loginfo('ready')
		rospy.spin()
		
		print "shutting down"

def nothing():
	import rospy
	rospy.init_node('benk',anonymous=True,disable_signals=True)
	import roslib
	roslib.load_manifest('tf')
	import tf.transformations as tft
	import numpy
	from brett2.PR2 import PR2

	q = numpy.array([0.0037790022003,0.0621387511329,0.624889472514,0.778228364249])
	p = numpy.array([0.516622185707,0.0262447148561,0.845655560493])
