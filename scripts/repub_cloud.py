#!/usr/bin/env python

import roslib
roslib.load_manifest("google_goggles")
import rospy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray
from google_goggles_msgs.msg import ObjectReferenceData

class CloudRepub(object):
	def __init__(self):
		self.sub1a = rospy.Subscriber('/camera/depth_registered/points_filtered', PointCloud2, self.cb1)
		self.sub1b = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.cb1)
		self.pub1 = rospy.Publisher('cloud',PointCloud2)
		
		self.sub2 = rospy.Subscriber('object_grasps', PoseArray, self.cb2)
		self.pub2 = rospy.Publisher('pose_array',PoseArray)
		
		self.sub3 = rospy.Subscriber('object_ref_data',ObjectReferenceData,self.cb3)
		self.pub3 = rospy.Publisher('ref_object',PointCloud2)
	
	def cb1(self,msg):
		rospy.loginfo('republishing cloud')
		#msg.header.frame_id = '/base_link'
		msg.header.stamp = rospy.Time.now()
		msg.header.stamp = rospy.Time(0)
		self.pub1.publish(msg)
	
	def cb2(self,msg):
		rospy.loginfo('republishing pose array')
		msg.header.stamp = rospy.Time.now()
		self.pub2.publish(msg)
	
	def cb3(self,msg):
		rospy.loginfo('publishing cloud from ref data')
		new_msg = msg.object.cloud
		self.pub3.publish(new_msg)

if __name__ == '__main__':
	rospy.init_node('repub_cloud',anonymous=True)
	
	repub = CloudRepub()
	
	rospy.loginfo('ready')
	
	rospy.spin()