#!/usr/bin/env python

import roslib
roslib.load_manifest('google_goggles')
import rospy
from geometry_msgs.msg import PoseArray

import rosbag
import os.path

import argparse

def main():
	rospy.init_node('display_grasps',anonymous=True)
	
	parser = argparse.ArgumentParser()
	
	parser.add_argument("name")
	parser.add_argument("selection",nargs='*',type=int)
	parser.add_argument('--frame')
	
	myargv = rospy.myargv()
	del myargv[0]
	
	args = parser.parse_args(myargv)
	print args
	
	pub = rospy.Publisher('grasp_poses',PoseArray)
	
	name = args.name
	print name
	rospy.loginfo('loading object %s', name)
	ref_pc_dir = roslib.packages.get_pkg_subdir('google_goggles','data/points/ref')
	filename = os.path.join(ref_pc_dir, name + '.bag')
	
	bag = rosbag.Bag(filename)
	
	grasps = None
	grasp_poses = None
	for topic, msg, t in bag.read_messages():
		if msg._type == 'google_goggles_msgs/ObjectReferenceData':
			rospy.loginfo('Got reference data on topic %s',topic)
			grasps = msg.grasps
		elif msg._type == 'geometry_msgs/PoseArray':
			grasp_poses = msg
	
	if grasps:
		grasp_poses = PoseArray()
		grasp_poses.poses = [grasp.grasp_pose for grasp in grasps]
		print [grasp.success_probability for grasp in grasps]
		
	
	if args.selection:
		grasp_poses.poses = [grasp_poses.poses[i] for i in args.selection]
	
	grasp_poses.header.stamp = rospy.Time.now()
	if args.frame:
		grasp_poses.header.frame_id = args.frame
	
	rospy.loginfo('Publishing %d poses in frame %s',len(grasp_poses.poses),grasp_poses.header.frame_id)
	pub.publish(grasp_poses)


if __name__ == '__main__':
	main()