#!/usr/bin/env python

import roslib
roslib.load_manifest("google_goggles")
import rospy

import rosbag

import sys, os, time, tempfile, os.path
from optparse import OptionParser
from collections import defaultdict

from objreco import GoogleGoggles

from image_tester import ImageTester

from object_loader import ObjectLoader

from grasper import Grasper

from math import *
import numpy

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

from threading import Thread

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


if __name__ == "__main__":
	parser = OptionParser()
	parser.add_option("--test",dest="test",action="store_true",default=False)
	
	parser.set_defaults(ros=False)
	parser.add_option("--ros",dest="ros",action="store_true")
	parser.add_option("--no-ros",dest="ros",action="store_false")
	
	parser.add_option("--fake-alignment",action="store_true",default=False)
	parser.add_option("--fake-align",dest='fake_alignment',action="store_true")
	
	parser.add_option("--no-images",action="store_true",default=False)
	
	parser.set_defaults(window=False)
	parser.add_option("--window",dest='window',action="store_true")
	parser.add_option("--no-window",dest='window',action="store_false")
	
	parser.add_option("--no-google",action="store_true",default=False)
	parser.add_option("--fake-google")
	
	parser.add_option("-t","--test-files", action="store_true",default=False, help="use files")
	
	parser.add_option("--label", help='label to test against')
	parser.add_option("--save-results",action="store_true",default=False)
	parser.add_option("--save-results-to")
	
	parser.add_option("-l", "--learn",action="append",default=[],
					help="learn image FILE",metavar="FILE")
	parser.add_option("--learn-image",help='learn image from camera',metavar="LABEL")
	parser.add_option("--save-image",help='save image with prefix PREFIX',metavar='PREFIX')
	
	parser.add_option("--max",type='int',default=0)
	parser.add_option("--single",action="store_const",dest="max",const=1)
	
	parser.add_option("-i","--interval",type="float",default=30)
	parser.add_option("--image-interval",type="float",default=4.5)
	
	parser.add_option('-s',"--size",dest="crop_size",help="image crop size WxH");
	
	parser.add_option("-c","--center",dest="crop_center",action="store_true",default=False);
	parser.add_option("-o","--offset",dest="crop_offset")
	
	parser.add_option("-g","--grasp",action="store_true",default=False, help="Run grasping");
	
	(options, args) = parser.parse_args(rospy.myargv())
	
	if not options.image_interval:
		options.image_interval = options.interval
	
	if options.save_results_to:
		options.save_results = True
	
	if options.learn_image and not options.max == 1:
		ans = raw_input('Are you sure you want to learn images with label {0}? '.format(options.learn_image))
		if ans != 'y' and ans != 'Y':
			print 'ok, exiting'
			exit(1)
		pass #options.max=1
	
	#print options
	#print args
	
	learnfiles = options.learn
	testfiles = []
	
	if options.test_files:
		if not args:
			parser.error("No files given for testing!")
		testfiles = args
	elif options.learn:
		learnfiles = learnfiles + args
	
	if learnfiles:
		results_file = None
		if options.save_results:
			results_file_name = 'results_' + time.strftime(TIME_FORMAT,time.localtime()) + '.txt'
			results_file = open(results_file_name,'w')
		for (idx,learnfile) in enumerate(learnfiles):
			if options.label:
				label = options.label
			else:
				label = learnfile.split("_")[0]
			print "learning {0} from file {1} ({2}/{3})".format(label,learnfile,idx+1,len(learnfiles))
			res = GoogleGoggles.learn(learnfile,label)
			if results_file:
				if res['status'] == 'SUCCESS':
					results_string = format_results('LEARN','SUCCESS',label,res['image_label'],time.localtime(),learnfile)
				else:
					results_string = format_results('LEARN','FAILURE',label,'~',time.localtime(),learnfile)
				results_file.write(results_string+'\n')
			print res
			if idx != len(learnfiles)-1: time.sleep(5)
		if results_file: results_file.close()
	
	if testfiles:
		results_file = None
		if options.save_results:
			results_file_name = 'results_' + time.strftime(TIME_FORMAT,time.localtime()) + '.txt'
			results_file = open(results_file_name,'w')
		total_successes = 0
		total_tests = 0
		label_successes = defaultdict(int)
		label_total = defaultdict(int)
		for (idx,testfile) in enumerate(testfiles):
			label = (testfile.split("/")[-1]).split("_")[0]
			print "testing file {0} ({1}/{2})".format(testfile,idx+1,len(testfiles))
			res = GoogleGoggles.match(testfile)
			success = res['image_label'] == label
			if success:
				print 'success!'
				total_successes += 1
				label_successes[label] = label_successes[label] + 1
				results_string = format_results('TEST','SUCCESS',label,res['image_label'],time.localtime(),testfile)
			else:
				print 'failure :('
				results_string = format_results('TEST','FAILURE',label,res['image_label'],time.localtime(),testfile)
			label_total[label] = label_total[label] + 1
			total_tests += 1
			if idx != len(testfiles)-1: time.sleep(5)
			if results_file:
				results_file.write(results_string+'\n')
		print 'successes: {0}/{1}'.format(total_successes,total_tests)
		for label in label_successes.keys():
			print "  {0}: {1}/{2}".format(label,label_successes[label],label_total[label])
		if results_file: results_file.close()
	elif options.ros or options.learn_image:
		rospy.init_node('cloud_grasping', anonymous=True)
		
		if options.grasp:
			rospy.loginfo('starting pr2')
			pr2 = PR2.create()
			pr2.grips.open()
			pr2.arms.goto_posture('side')
			#gui = RaveGUI(pr2.env)
			#gui.start()
		
		if not options.no_images:
			rospy.loginfo("Starting ImageTester")
			image_tester = ImageTester(options,image_topic='/prosilica/image_rect_color',image_label_topic='object_name')
		
		rospy.loginfo("Starting ObjectLoader")
		object_loader = ObjectLoader(object_name_topic='object_name',grasp_poses_topic='grasp_poses',grasper=options.grasp,fake_alignment=options.fake_alignment,interval=options.interval)
		
		if options.grasp:
			pass
			#rospy.loginfo("Starting Grasper")
			#grasper = Grasper(pr2,grasp_pose_array_topic='grasp_poses',table_height_topic='table_height')
			#grasper = Grasper(pr2,grasp_pose_array_topic='grasp_poses',table_height_topic='table_height',object_point_cloud_topic='aligned_object')
		
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
