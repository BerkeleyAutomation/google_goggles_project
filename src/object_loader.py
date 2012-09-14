import roslib
roslib.load_manifest("google_goggles")
import rospy

import rosbag

import sys, os, time, tempfile, os.path
from optparse import OptionParser
from collections import defaultdict

from objreco import GoogleGoggles

from math import *
import numpy

import cv
from google_goggles_msgs.msg import ObjectReferenceData
import google_goggles_srvs.srv
from geometry_msgs.msg import *
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32, String, Header
from cv_bridge import CvBridge, CvBridgeError
from pcl.msg import PolygonMesh
import tf
import tf.transformations as tft

class ObjectLoader:
	def __init__(self,object_name_topic='object_name',grasp_poses_topic='grasp_poses',fake_alignment=True):
		self.ref_cloud_pub = rospy.Publisher('ref_cloud',PointCloud2,latch=True)
		self.ref_data_pub = rospy.Publisher('ref_data',ObjectReferenceData,latch=True)
		
		self.cloud_to_mesh_srv_name = 'create_mesh'
		self.cloud_to_mesh_srv = rospy.ServiceProxy(self.cloud_to_mesh_srv_name, google_goggles_srvs.srv.CreateMesh)  
		
		self.fake_alignment = fake_alignment
		self.align_object_srv_name = 'align_object'
		self.align_object_srv = None
		if not self.fake_alignment:
			rospy.wait_for_service(self.align_object_srv_name)
			self.align_object_srv = rospy.ServiceProxy(self.align_object_srv_name,google_goggles_srvs.srv.AlignObject)
		else:
			self.fake_align_pub = rospy.Publisher('aligned_object',PointCloud2)
		
		self.grasp_poses_topic = grasp_poses_topic
		self.grasp_poses_pub = rospy.Publisher(self.grasp_poses_topic,PoseArray,latch=True)
		
		self.object_name_sub = rospy.Subscriber(object_name_topic,String,self.object_name_callback)
	
	def object_name_callback(self,msg):
		name = msg.data
		rospy.loginfo('loading object %s', name)
		ref_pc_dir = roslib.packages.get_pkg_subdir('google_goggles','data/points/ref')
		filename = os.path.join(ref_pc_dir, name + '.bag')
		
		ref_data = ObjectReferenceData()
		
		ref_object = None
		ref_cloud = None
		ref_grasps = []
		ref_grasp_poses = []
		
		rospy.loginfo('Loading bag file...')
		bag = rosbag.Bag(filename)
		rospy.loginfo('Searching bag file for data')
		for topic,msg,t in bag.read_messages():
			topic = topic.split('/')[-1]
			if msg._type == 'google_goggles_msgs/ObjectReferenceData': #if topic.endswith('object_ref_data'):
				rospy.loginfo('Got reference data on topic %s',topic)
				#if ref_object:
				#	rospy.logerr('Bag file has multiple objects!')
				#	return False
				ref_object = msg.object
				ref_grasps += msg.grasps
			elif msg._type == 'pcl/PolygonMesh': #if topic.endswith('object_mesh'):
				rospy.loginfo('Got mesh on topic %s',topic)
				#if ref_object:
				#	rospy.logerr('Bag file has multiple objects!')
				#	return False
				ref_object = msg
			elif msg._type == 'sensor_msgs/PointCloud2':
				rospy.loginfo('Got cloud on topic %s',topic)
				ref_cloud = msg
			elif msg._type == 'geometry_msgs/PoseArray':
				ref_grasp_poses += msg.poses
		
		if ref_object is None:
			try:
				rospy.loginfo('Converting cloud to mesh')
				rospy.wait_for_service(self.cloud_to_mesh_srv_name)
				ref_object = self.cloud_to_mesh_srv(ref_cloud)
			except rospy.ServiceException, e:
				rospy.logerr("Create mesh service error: %s",str(e))
				return False
		
		try:
			rospy.loginfo('Aligning object...')
			if self.fake_alignment:
				rospy.loginfo("Publishing fake-aligned object")
				self.fake_align_pub.publish(ref_object.cloud)
				object_pose = tft.identity_matrix()
				object_pose_header = Header()
				object_pose_header.frame_id = ref_object.cloud.header.frame_id
				object_pose_header.stamp = rospy.Time.now()
			else:
				align_object_response = self.align_object_srv(ref_object)
				object_pose_msg = align_object_response.pose
				#print "Aligned object pose: ", object_pose_msg
				object_pose_header = object_pose_msg.header
				object_pose_q = numpy.array([ \
						object_pose_msg.pose.orientation.x,
						object_pose_msg.pose.orientation.y,
						object_pose_msg.pose.orientation.z,
						object_pose_msg.pose.orientation.w])
				object_pose_p = numpy.array([ \
						object_pose_msg.pose.position.x,
						object_pose_msg.pose.position.y,
						object_pose_msg.pose.position.z])
				rot = numpy.mat(tft.quaternion_matrix(object_pose_q))
				trans = numpy.mat(tft.translation_matrix(object_pose_p))
				object_pose = trans * rot
		except rospy.ServiceException, e:
			rospy.logerr("Align Object service error: %s",str(e))
			return False
			
		#apply tf grasp
		grasps = []
		for grasp in ref_grasps:
			grasp_pose_msg = grasp.grasp_pose
			#print "pose msg", grasp_pose_msg
			grasp_pose_q = numpy.array([ \
					grasp_pose_msg.orientation.x,
					grasp_pose_msg.orientation.y,
					grasp_pose_msg.orientation.z,
					grasp_pose_msg.orientation.w])
			grasp_pose_p = numpy.array([ \
					grasp_pose_msg.position.x,
					grasp_pose_msg.position.y,
					grasp_pose_msg.position.z])
			rot = numpy.mat(tft.quaternion_matrix(grasp_pose_q))
			trans = numpy.mat(tft.translation_matrix(grasp_pose_p))
			ref_grasp_pose = trans * rot
			grasp_pose = object_pose * ref_grasp_pose
			grasps.append((grasp_pose,grasp.success_probability))
		
		sorted_grasps = [g[0] for g in sorted(grasps,key=lambda g: g[1])]
		
		grasp_pose_array = PoseArray()
		grasp_pose_array.header = object_pose_header
		for grasp in sorted_grasps:
			grasp_pose_msg = Pose()
			q = tft.quaternion_from_matrix(grasp)
			p = tft.translation_from_matrix(grasp)
			grasp_pose_msg.orientation.x = q[0]
			grasp_pose_msg.orientation.y = q[1]
			grasp_pose_msg.orientation.z = q[2]
			grasp_pose_msg.orientation.w = q[3]
			grasp_pose_msg.position.x = p[0]
			grasp_pose_msg.position.y = p[1]
			grasp_pose_msg.position.z = p[2]
			grasp_pose_array.poses.append(grasp_pose_msg)
			
		rospy.loginfo('Publishing grasps')
		self.grasp_poses_pub.publish(grasp_pose_array)
		rospy.loginfo('Object loading complete')