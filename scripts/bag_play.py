#!/usr/bin/env python

import roslib
roslib.load_manifest('google_goggles')
import rospy
from rosbag import Bag

from tf.msg import tfMessage

import pandas as pd
import numpy as np

import optparse

if __name__ == '__main__':
	rospy.init_node('bag_play',anonymous=True)
	
	parser = optparse.OptionParser()
	
	parser.add_option('-l','--loop',action='store_true',default=False)
	parser.add_option('--tf',action='store_true',default=False,dest='tf_only')
	parser.add_option('--no-tf',action='store_true',default=False)
	parser.add_option('-n','--num-loops',action='store',type=int,default=1)
	
	(options,args) = parser.parse_args(rospy.myargv())
	
	bag_file = args[1]
	
	bag = Bag(bag_file)
	
	rospy.loginfo('loading messages from bag')
	
	df = pd.DataFrame([])
	df['t']= []
	df['topic'] = []
	df['msg'] = []
	
	for topic,msg,t in bag:
		if topic in ['tf','/tf'] and options.no_tf:
			continue
		elif topic not in ['tf','/tf'] and options.tf_only:
			continue
		s = pd.Series([t.to_sec(),topic,msg],index=['t','topic','msg'])
		df = df.append(s,ignore_index=True)
	
	df.sort_index(by='t')
	df['t'] = df['t'] - df['t'][0]
	
	pubs = {}
	
	rospy.loginfo('publishing %d messages',len(df))
	
	loop = options.loop
	num_loops = options.num_loops
	
	orig_start_time = rospy.Time.now()
	start_time = orig_start_time
	
	loop_num = 0
	while (loop or loop_num < num_loops) and not rospy.is_shutdown():
		loop_num += 1
		print 'loop #%d (%f)' % (loop_num,(rospy.Time.now()-start_time).to_sec())
		start_time = rospy.Time.now()
		now = start_time
		current_offset = 0
		offset_diffs = pd.Series(np.nan,index=df.index)
		for idx in df.index:
			if rospy.is_shutdown():
				break
			offset = df.get_value(idx, 't')
			if offset > current_offset:
				rospy.sleep(current_offset)
				now = rospy.Time.now()
				current_offset = (now - start_time).to_sec()
			actual_offset = (rospy.Time.now() - start_time).to_sec()
			offset_diff = actual_offset - current_offset
			offset_diffs[idx] = offset_diff
			topic = df.get_value(idx,'topic')
			msg = df.get_value(idx,'msg')
			if msg._type == 'tf/tfMessage' and not options.no_tf:
				for i in xrange(len(msg.transforms)):
					msg.transforms[i].header.stamp = now
			elif hasattr(msg,'header') and not options.tf_only:
				msg.header.stamp = now
			
			if not pubs.has_key(topic):
				pubs[topic] = rospy.Publisher(topic,type(msg))
			pubs[topic].publish(msg)
		print 'avg diff: %f\tmax diff: %f' % (offset_diffs.mean(), offset_diffs.max())