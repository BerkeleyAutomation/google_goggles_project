import roslib
roslib.load_manifest("google_goggles")
import rospy

import rosbag

import sys, os, time, tempfile, os.path
from collections import defaultdict

from math import *
import numpy
import random

from std_msgs.msg import Float32, Float64, Bool, Int32
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import tf
import tf.transformations as tft

import google_goggles_srvs.srv

from brett2.PR2 import PR2, IKFail
import openravepy

from actionlib import SimpleActionClient, SimpleGoalState
from arm_navigation_msgs.msg import *

from pr2_simple_motions_srvs.srv import *
from pr2_common_action_msgs.msg import *
from pr2_controllers_msgs.msg import *
from std_srvs.srv import Empty
import actionlib

import struct

from threading import Thread

class RaveGUI(Thread):
    
    def __init__(self, env):
        Thread.__init__(self)
        self.env = env
        
    def run(self):
        self.env.SetViewer('qtcoin')
        while not rospy.is_shutdown():
            rospy.sleep(.01)

class Grasper:
	pr2 = None
	
	def __init__(self,
			table_height=None,
			grasp_pose_topic=None,
			grasp_pose_array_topic=None,
			table_height_topic=None,
			object_mesh_topic=None,
			object_point_cloud_topic=None,
			object_pose_topic=None,
			always_publish_pose=False,
			select_grasp=None,
			select_grasps_after=None,
			shuffle_poses=True):
		self.pr2 = PR2.create()
		
		rospy.loginfo("reseting")
		self.pr2.rarm.goto_posture('side')
		self.pr2.rgrip.open()
		
		
		self.base_frame = 'base_footprint'
		#self.tool_frame = 'r_gripper_tool_frame'
		self.tool_frame = 'r_wrist_roll_link'
		
		#self.move_arm = SimpleActionClient('move_right_arm', MoveArmAction)
		
		#set up publishers
		self.pregrasp_pose_pub = rospy.Publisher('pregrasp_pose',PoseStamped,latch=True);
		self.grasp_pose_pub = rospy.Publisher('grasp_pose',PoseStamped,latch=False);
		self.lift_pose_pub = rospy.Publisher('lift_pose',PoseStamped,latch=True);
		self.lift_rot_pose_pub = rospy.Publisher('lift_pose_rot',PoseStamped,latch=True);
		self.grasp_pose_array_pub = rospy.Publisher('grasp_pose_array',PoseArray,latch=True);
		
		self.grasp_num_pub = rospy.Publisher('grasp_num',Int32,latch=False)
		self.grasp_quality_pub = rospy.Publisher('grasp_quality',Float64,latch=False)
		self.success_pub = rospy.Publisher('success',Bool,latch=False)
		
		self.tracker_pause_pub = rospy.Publisher('tracker_pause',Bool);
		
		self.always_publish_pose = always_publish_pose
		self.select_grasp = select_grasp
		self.select_grasps_after = select_grasps_after
		self.shuffle_poses = shuffle_poses
		
		#table height is used for collision checking grasps
		self.table_height = table_height
		self.table_height_topic = table_height_topic
		if self.table_height_topic:
			self.table_height_sub = rospy.Subscriber(self.table_height_topic, Float32, self.table_height_callback)
		
		#object mesh vs. point cloud options
		self.use_object_mesh = False
		self.object_mesh = None
		
		self.use_object_pose = False
		self.object_pose = None
		self.object_mesh_topic = object_mesh_topic
		if self.object_mesh_topic:
			self.use_object_mesh = True
			self.object_mesh_sub = rospy.Subscriber(object_mesh_topic,PolygonMesh,self.object_mesh_callback)
		
		self.object_point_cloud_topic = object_point_cloud_topic
		if self.object_point_cloud_topic:
			#print 'subscribing to ', object_point_cloud_topic
			self.use_object_mesh = True
			self.cloud_to_mesh_srv_name = 'create_mesh'
			self.cloud_to_mesh_srv = None
			self.object_point_cloud_sub = rospy.Subscriber(object_point_cloud_topic,PointCloud2,self.object_point_cloud_callback)
		
		self.object_pose_topic = object_pose_topic
		if self.object_pose_topic:
			self.use_object_pose = True
			self.object_pose_sub = rospy.Subscriber(object_pose_topic,self.object_pose_callback)
		
		if grasp_pose_topic:
			self.grasp_pose_sub = rospy.Subscriber(grasp_pose_topic,PoseStamped,self.grasp_pose)
		
		if grasp_pose_array_topic:
			self.grasp_pose_array_sub = rospy.Subscriber(grasp_pose_array_topic,PoseArray,self.grasp_pose_array)
	
	def grasp_pose_array(self,msg,qualities = None):
		#Upon receiving a list of grasps, we want to select one, test it, 
		# and if it doesn't work, go on to the next one
		#A grasp is given as pose of the gripper
		grasp_nums = list(xrange(len(msg.poses)))

		#we have options that allow us to select or eliminate grasps for testing
		if self.select_grasp is not None:
			print 'selecting grasp %d' % self.select_grasp
			msg.poses = [msg.poses[self.select_grasp]]
			grasp_nums = [grasp_nums[self.select_grasp]]
			if qualities:
				qualities = [qualities[self.select_grasp]]
		elif self.select_grasps_after is not None:
			print 'selecting grasps after %d' % self.select_grasps_after
			msg.poses = msg.poses[self.select_grasps_after:]
			grasp_nums = grasp_nums[self.select_grasps_after:]
			if qualities:
				qualities = qualities[self.select_grasps_after:]
		
		#optionally shuffle the grasps
		if self.shuffle_poses:
			print "shuffling"
			if qualities:
				l = zip(msg.poses,qualities,grasp_nums)
				random.shuffle(l)
				msg.poses,qualities,grasp_nums = zip(*l)
			else:
				l = zip(msg.poses,grasp_nums)
				random.shuffle(l)
				msg.poses,grasp_nums = zip(*l)

		#The table height may interfere with all of the grasps, so if none of them work,
		# shift the table down and try again
		max_shift = -0.05
		shift_steps = 5
		result = None
		for shift_step in xrange(shift_steps):
			shift = max_shift * shift_step / shift_steps
			if shift_step != 0:
				print 'shifting table down by %f' % (-shift)
			#test each pose in turn
			for idx, pose in enumerate(msg.poses):
				grasp_num = grasp_nums[idx]
				pose_stamped = PoseStamped()
				pose_stamped.header = msg.header
				pose_stamped.pose = pose
				quality = None
				quality_str = ''
				if qualities:
					quality = qualities[idx]
					quality_str = ' q=%f' % quality
				print 'Testing pose #%d %d/%d%s' % (grasp_num,idx+1,len(msg.poses),quality_str)
				result = self.grasp_pose(pose_stamped,delete_object_store=False,table_shift=shift,quality=quality)
				if result is not None:
					self.grasp_num_pub.publish(grasp_num)
					if quality is not None:
						self.grasp_quality_pub.publish(quality)
					self.success_pub.publish(result)
					print pose
					break
			else:
				print 'No grasp was successful :('
			if result is not None:
					break
		self.object_mesh = None
		self.object_pose = None
		return result

	def grasp_pose(self,msg,delete_object_store=True,table_shift=0,quality=None):
		if not self.table_height:
			rospy.loginfo('waiting for table height...')
			rospy.wait_for_message(self.table_height_topic,Float32)
			print 'got it!'
		
		(tf_trans,tf_rot) = self.pr2.tf_listener.lookupTransform(msg.header.frame_id,self.base_frame,rospy.Time(0))
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
		
		lift_pose = tft.translation_matrix(numpy.array([0,0,0.3])) * pose
		
		lift_pose_rot_only = numpy.mat(numpy.copy(lift_pose))
		lift_pose_rot_only[0:3,3] = 0
		
		lift_pose_x_axis = list((lift_pose_rot_only * numpy.matrix([1,0,0,1]).transpose())[0:3].flat)
		
		lift_rot_pose = numpy.mat(tft.quaternion_matrix(tft.quaternion_about_axis(pi, lift_pose_x_axis))) * lift_pose_rot_only
		lift_rot_pose[0:3,3] = lift_pose[0:3,3]
		
		if self.always_publish_pose:
			msg = PoseStamped()
			msg.header.stamp = rospy.Time.now()
			msg.header.frame_id = self.base_frame
			mq,mp = (tft.quaternion_from_matrix(pose),tft.translation_from_matrix(pose)) 
			msg.pose.orientation.x = mq[0]
			msg.pose.orientation.y = mq[1]
			msg.pose.orientation.z = mq[2]
			msg.pose.orientation.w = mq[3]
			msg.pose.position.x = mp[0]
			msg.pose.position.y = mp[1]
			msg.pose.position.z = mp[2]
			self.grasp_pose_pub.publish(msg)
		
		for body in self.pr2.env.GetBodies():
			if body.GetName() == 'table':
				self.pr2.env.Remove(body)
				break
		
		#rospy.loginfo('adding table to openrave env')
		body = openravepy.RaveCreateKinBody(self.pr2.env,'')
		body.SetName('table')
		body.InitFromBoxes(numpy.array([[pose_x,pose_y,self.table_height/2 + table_shift,0.25,1,self.table_height/2]]),True)
		self.pr2.env.AddKinBody(body)
		
		if self.use_object_mesh:
			if not self.object_mesh:
				if self.object_mesh_topic:
					rospy.loginfo('Waiting for object mesh...')
				else:
					rospy.loginfo('Waiting for object point cloud...')
				rate = rospy.Rate(0.1)
				while not self.object_mesh:
					rate.sleep()
			for body in self.pr2.env.GetBodies():
				if body.GetName() == 'object':
					self.pr2.env.Remove(body)
					break
			if self.use_object_pose:
				if not self.object_pose:
					rospy.loginfo('Waiting for object pose...')
					rate = rospy.Rate(0.1)
					while not self.object_mesh:
						rate.sleep()
				body.SetTransform(self.object_pose)
			rospy.loginfo('adding object to openrave env')
			body = openravepy.RaveCreateKinBody(self.pr2.env,'')
			body.SetName('object')
			body.InitFromTrimesh(self.object_mesh,True)
			self.pr2.env.AddKinBody(body)
			if delete_object_store:
				self.object_mesh = None
				self.object_pose = None
			
			#self.pr2.env.SetViewer('qtcoin')
			#gui = RaveGUI(self.pr2.env)
			#gui.start()
		
		qt,pt = (tft.quaternion_from_matrix(first_pose),tft.translation_from_matrix(first_pose)) 
		#print "first_pose:"
		#print numpy.array(first_pose)
		
		filteropts = openravepy.openravepy_int.IkFilterOptions.CheckEnvCollisions
		filteropts = filteropts | openravepy.openravepy_int.IkFilterOptions.IgnoreSelfCollisions
		
		invalid = False
		if not self.pr2.rarm.check_pose_matrix(numpy.array(first_pose),self.base_frame,self.tool_frame,filter_options=filteropts):
			print "IK invalid: pregrasp pose"
			invalid = True
			#return False
		if not self.pr2.rarm.check_pose_matrix(numpy.array(pose),self.base_frame,self.tool_frame,filter_options=filteropts):
			print "IK invalid: grasp pose"
			invalid = True
			#return False
		if not self.pr2.rarm.check_pose_matrix(numpy.array(lift_pose),self.base_frame,self.tool_frame,filter_options=filteropts):
			print "IK invalid: lift pose"
			invalid = True
			#return False
		if invalid:
			return None
		
		rospy.loginfo("IK valid, proceeding with grasp")
		
		self.tracker_pause_pub.publish(True)
		
		
		rospy.loginfo("opening gripper")
		self.pr2.rgrip.open()
		#rospy.sleep(1.5)
		
		rospy.loginfo("moving arm")
		
		pose_array = PoseArray()
		pose_array.header.stamp = rospy.Time.now()
		pose_array.header.frame_id = self.base_frame
		
		pub_poses = [ \
			(self.pregrasp_pose_pub,first_pose), \
			(self.grasp_pose_pub,pose), \
			(self.lift_pose_pub,lift_pose), \
			(self.lift_rot_pose_pub,lift_rot_pose)]
		
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
			self.pr2.rarm.goto_pose_matrix(numpy.array(first_pose),self.base_frame,self.tool_frame,filter_options=filteropts)
			rospy.sleep(10)
		except IKFail, e:
			print 'IK failed for first pose:',e
			self.pr2.rgrip.open()
			self.pr2.rarm.goto_posture('side')
			return None
		
		#print "second pose:"
		#print numpy.array(pose)
		try:
			rospy.loginfo("Moving to grasp")
			self.pr2.rarm.goto_pose_matrix(numpy.array(pose),self.base_frame,self.tool_frame,filter_options=filteropts)
			rospy.sleep(5)
		except IKFail, e:
			print 'IK failed for second pose:',e
			self.pr2.rgrip.open()
			self.pr2.rarm.goto_posture('side')
			return None
		
		rospy.loginfo("closing gripper")
		max_effort=-1
		#max_effort = 40
		self.pr2.rgrip.close(max_effort=max_effort); rospy.sleep(3)
		
		#print "lifting:"
		#print numpy.array(lift_pose)
		try:
			rospy.loginfo("lifting")
			self.pr2.rarm.goto_pose_matrix(numpy.array(lift_pose),self.base_frame,self.tool_frame,filter_options=filteropts)
			rospy.sleep(5)
		except IKFail, e:
			print 'IK failed for lifting:',e
			self.pr2.rgrip.open()
			self.pr2.rarm.goto_posture('side')
			return None
		
		try:
			rospy.loginfo("rotating")
			self.pr2.rarm.goto_pose_matrix(numpy.array(lift_rot_pose),self.base_frame,self.tool_frame,filter_options=filteropts)
			rospy.sleep(10)
		except IKFail, e:
			print 'IK failed for lifting:',e
			self.pr2.rgrip.open()
			self.pr2.rarm.goto_posture('side')
			return None
		
		jm = self.pr2.get_last_joint_message()
		g_pos = jm.position[jm.name.index('r_gripper_joint')]
		if g_pos < 0.005:
			print 'Failure: gripper is closed!'
			self.pr2.rgrip.open()
			self.pr2.rarm.goto_posture('side')
			return False
		
		print 'Success!'
		
		#rospy.loginfo("reseting")
		#self.pr2.rarm.goto_posture('side')
		#self.pr2.rgrip.open()
		
		return True

	def table_height_callback(self,msg):
		if not self.table_height: rospy.loginfo("got table height: %f" % msg.data)
		self.table_height = msg.data
	
	def object_point_cloud_callback(self,msg):
		print 'pc cb'
		if not self.cloud_to_mesh_srv:
			rospy.loginfo('Waiting for create mesh service')
			rospy.wait_for_service(self.cloud_to_mesh_srv_name)
			self.cloud_to_mesh_srv = rospy.ServiceProxy(self.cloud_to_mesh_srv_name, google_goggles_srvs.srv.CreateMesh)
		response = self.cloud_to_mesh_srv(msg)
		self.object_mesh_callback(response.mesh)
		
	def object_mesh_callback(self,mesh):
		print 'mesh cb'
		if mesh.cloud.height != 1:
			rospy.logerr('2D point cloud!')
			return
		elif mesh.cloud.width <= 0:
			rospy.logerr('Invalid width!')
			return
		elif mesh.cloud.fields[0].name != "x" or mesh.cloud.fields[1].name != "y" or mesh.cloud.fields[2].name != "z":
			rospy.logerr("point cloud has bad fields!")
			return
		elif mesh.cloud.fields[0].datatype != sensor_msgs.msg.PointField.FLOAT32:
			rospy.logerr("point cloud is not float32!")
			return
		
		v = numpy.empty((mesh.cloud.width,3))
		t = numpy.empty((len(mesh.polygons),3))
		
		print "converting vertices"
		
		for i in xrange(mesh.cloud.width):
			offset = mesh.cloud.point_step * i
			for j in xrange(3):
				v[i,j] = struct.unpack_from('f',mesh.cloud.data,offset+j*4)[0]
		
		print 'converting indices'
		
		for i in xrange(len(mesh.polygons)):
			for j in xrange(3):
				t[i,j] = mesh.polygons[i].vertices[j]
		
		self.object_mesh = openravepy.openravepy_int.TriMesh(v,t)
		print 'done'
		
		
		
	def object_pose_callback(self,msg):
		print 'object pose cb'
		(tf_trans,tf_rot) = self.pr2.tf_listener.lookupTransform(msg.header.frame_id,self.base_frame,rospy.Time(0))
		msg_tf = numpy.mat(numpy.dot(tft.translation_matrix(tf_trans),tft.quaternion_matrix(tf_rot)))
		
		q = numpy.array([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
		p = numpy.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
		rot = numpy.mat(tft.quaternion_matrix(q))
		trans = numpy.mat(tft.translation_matrix(p))
		self.object_pose = msg_tf * trans * rot
