import roslib
roslib.load_manifest("google_goggles")
import rospy

import rosbag

import sys, os, time, tempfile, os.path
from collections import defaultdict

from math import *
import numpy

from std_msgs.msg import Float32
from geometry_msgs.msg import *
import tf
import tf.transformations as tft

from brett2.PR2 import PR2, IKFail
import openravepy

from actionlib import SimpleActionClient, SimpleGoalState
from arm_navigation_msgs.msg import *

from pr2_simple_motions_srvs.srv import *
from pr2_common_action_msgs.msg import *
from pr2_controllers_msgs.msg import *
from std_srvs.srv import Empty
import actionlib

class Grasper:
	pr2 = None
	
	def __init__(self,pr2=None,grasp_pose_topic=None,grasp_pose_array_topic=None,table_height=None,table_height_topic=None):
		self.pr2 = pr2
		if not self.pr2:
			self.pr2 = PR2()
		
		rospy.loginfo("reseting")
		self.pr2.rarm.goto_posture('side')
		self.pr2.rgrip.open()
		
		#self.move_arm = SimpleActionClient('move_right_arm', MoveArmAction)
		
		self.pregrasp_pose_pub = rospy.Publisher('pregrasp_pose',PoseStamped,latch=True);
		self.grasp_pose_pub = rospy.Publisher('grasp_pose',PoseStamped,latch=True);
		self.lift_pose_pub = rospy.Publisher('lift_pose',PoseStamped,latch=True);
		self.grasp_pose_array_pub = rospy.Publisher('grasp_pose_array',PoseArray,latch=True);
		
		self.table_height = table_height
		self.table_height_topic = table_height_topic
		if self.table_height_topic:
			self.table_height_sub = rospy.Subscriber(self.table_height_topic, Float32, self.table_height_callback)
		
		if grasp_pose_topic:
			self.grasp_pose_sub = rospy.Subscriber(grasp_pose_topic,PoseStamped,self.grasp_pose_callback)
		
		if grasp_pose_array_topic:
			self.grasp_pose_array_sub = rospy.Subscriber(grasp_pose_array_topic,PoseArray,self.grasp_pose_array_callback)
	
	def grasp_pose_array_callback(self,msg):
		for idx, pose in enumerate(msg.poses):
			pose_stamped = PoseStamped()
			pose_stamped.header = msg.header
			pose_stamped.pose = pose
			print 'Testing pose %d/%d' % (idx+1,len(msg.poses))
			result = self.grasp_pose_callback(pose_stamped)
			if result:
				print 'Success!'
				break
		else:
			print 'No grasp was successful :('

	def grasp_pose_callback(self,msg):
		if not self.table_height: return
		base_frame = 'base_footprint'
		#tool_frame = 'r_gripper_ tool_frame'
		tool_frame = 'r_wrist_roll_link'
		
		(tf_trans,tf_rot) = self.pr2.tf_listener.lookupTransform(msg.header.frame_id,base_frame,rospy.Time(0))
		msg_tf = numpy.mat(numpy.dot(tft.translation_matrix(tf_trans),tft.quaternion_matrix(tf_rot)))
		
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
		first_pose = pose * tft.translation_matrix([-init_dist,0,0])
		#first_pose = pose * tft.translation_matrix(numpy.array([0,0,0.3]))
		
		lift_pose = pose * tft.translation_matrix(numpy.array([0,0,0.3]))
		
		for body in self.pr2.env.GetBodies():
			if body.GetName() == 'table': break
		else:
			rospy.loginfo('adding table to openrave env')
			body = openravepy.RaveCreateKinBody(self.pr2.env,'')
			body.SetName('table')
			body.InitFromBoxes(numpy.array([[pose_x,pose_y,self.table_height/2,0.25,1,self.table_height/2]]),False)
			self.pr2.env.AddKinBody(body)
		
		
		qt,pt = (tft.quaternion_from_matrix(first_pose),tft.translation_from_matrix(first_pose)) 
		#print "first_pose:"
		#print numpy.array(first_pose)
		
		filteropts = openravepy.openravepy_int.IkFilterOptions.CheckEnvCollisions
		filteropts = filteropts | openravepy.openravepy_int.IkFilterOptions.IgnoreSelfCollisions
		
		invalid = False
		if not self.pr2.rarm.check_pose_matrix(numpy.array(first_pose),base_frame,tool_frame,filter_options=filteropts):
			print "IK invalid: pregrasp pose"
			invalid = True
			#return False
		if not self.pr2.rarm.check_pose_matrix(numpy.array(pose),base_frame,tool_frame,filter_options=filteropts):
			print "IK invalid: grasp pose"
			invalid = True
			#return False
		if not self.pr2.rarm.check_pose_matrix(numpy.array(lift_pose),base_frame,tool_frame,filter_options=filteropts):
			print "IK invalid: lift pose"
			invalid = True
			#return False
		if invalid:
			return False
		
		rospy.loginfo("IK valid, proceeding with grasp")
		
		
		rospy.loginfo("opening gripper")
		self.pr2.rgrip.open()
		#rospy.sleep(1.5)
		
		rospy.loginfo("moving arm")
		
		pose_array = PoseArray()
		pose_array.header.stamp = rospy.Time.now()
		pose_array.header.frame_id = base_frame
		
		pub_poses = [ \
			(self.pregrasp_pose_pub,first_pose), \
			(self.grasp_pose_pub,pose), \
			(self.lift_pose_pub,lift_pose)]
		
		for pub,pose_to_pub in pub_poses:
			msg = PoseStamped()
			msg.header = pose_array.header
			mq,mp = (tft.quaternion_from_matrix(pose_to_pub),tft.translation_from_matrix(pose_to_pub)) 
			msg.pose.orientation.x = mq[0]
			msg.pose.orientation.y = mq[1]
			msg.pose.orientation.z = mq[2]
			msg.pose.orientation.w = mq[3]
			msg.pose.position.x = mp[0]
			msg.pose.position.y = mp[1]
			msg.pose.position.z = mp[2]
			pose_array.poses.append(msg.pose)
			pub.publish(msg)
		
		self.grasp_pose_array_pub.publish(pose_array)
		
		try:
			rospy.loginfo("Going to pregrasp")
			self.pr2.rarm.goto_pose_matrix(numpy.array(first_pose),base_frame,tool_frame,filter_options=filteropts)
			rospy.sleep(10)
		except IKFail, e:
			print 'IK failed for first pose:',e
			self.pr2.rarm.goto_posture('side')
			self.pr2.rgrip.open()
			return False
		
		#print "second pose:"
		#print numpy.array(pose)
		try:
			rospy.loginfo("Moving to grasp")
			self.pr2.rarm.goto_pose_matrix(numpy.array(pose),base_frame,tool_frame,filter_options=filteropts)
			rospy.sleep(5)
		except IKFail, e:
			print 'IK failed for second pose:',e
			self.pr2.rarm.goto_posture('side')
			self.pr2.rgrip.open()
			return False
		
		rospy.loginfo("closing gripper")
		self.pr2.rgrip.close(); rospy.sleep(3)
		
		#print "lifting:"
		#print numpy.array(lift_pose)
		try:
			rospy.loginfo("lifting")
			self.pr2.rarm.goto_pose_matrix(numpy.array(lift_pose),base_frame,tool_frame,filter_options=filteropts)
			rospy.sleep(5)
		except IKFail, e:
			print 'IK failed for lifting:',e
			self.pr2.rarm.goto_posture('side')
			self.pr2.rgrip.open()
			return False
		
		#rospy.loginfo("reseting")
		#self.pr2.rarm.goto_posture('side')
		#self.pr2.rgrip.open()
		
		return True

	def table_height_callback(self,msg):
		if not self.table_height: rospy.loginfo("got table height: %f" % msg.data)
		self.table_height = msg.data