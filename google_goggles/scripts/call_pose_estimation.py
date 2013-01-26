#!/usr/bin/env python

import roslib
roslib.load_manifest('google_goggles')
import rospy

from google_goggles_msgs.msg import ObjectReferenceData
import google_goggles_srvs.srv
import os.path
import rosbag
import numpy
import tf.transformations as tft

def main():
	rospy.init_node('call_pose_estimation',anonymous=True)
	
	cloud_to_mesh_srv_name = 'create_mesh'
	cloud_to_mesh_srv = rospy.ServiceProxy(cloud_to_mesh_srv_name, google_goggles_srvs.srv.CreateMesh)
	
	align_object_srv_name = 'align_object'
	rospy.wait_for_service(align_object_srv_name)
	align_object_srv = rospy.ServiceProxy(align_object_srv_name,google_goggles_srvs.srv.AlignObject)
	
	name = rospy.myargv()[1]
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
			rospy.wait_for_service(cloud_to_mesh_srv_name)
			ref_object = cloud_to_mesh_srv(ref_cloud)
		except rospy.ServiceException, e:
			rospy.logerr("Create mesh service error: %s",str(e))
			last_call_time = rospy.Time.now()
			return False
	
	try:
		rospy.loginfo('Aligning object...')
		try:
			rospy.wait_for_service(align_object_srv_name)
			align_object_response = align_object_srv(ref_object)
		except Exception, e:
			print 'aligning failed',e
			return True
		object_pose_msg = align_object_response.pose
		rospy.loginfo("Aligned object pose: %s", object_pose_msg)
		aligned_object = align_object_response.aligned_object
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
		last_call_time = rospy.Time.now()
		return True

if __name__ == '__main__':
	main()