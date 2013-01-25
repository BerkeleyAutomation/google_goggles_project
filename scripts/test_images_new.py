#!/usr/bin/env python

import roslib
roslib.load_manifest('google_goggles')

from collections import defaultdict

import sys, os, os.path, random, time, math
from optparse import OptionParser
import pickle

import numpy

TIME_FORMAT = "%Y-%m-%d-T%H-%M-%S"

import os.path, time
import urllib, urllib2, json

from google_goggles_connector import GoogleGoggles
from image_testing import ImageTester

def main(argv):
	parser = OptionParser('%prog [OPTIONS]')
	parser.add_option('--test',action='store_true',default=False)
	
	parser.add_option('--name')
	parser.add_option('--save-dir',default='')
	
	parser.add_option('-d','--dir',dest='validation_dir')
	parser.add_option('-v','--validation-dir',default='.')
	parser.add_option('-t','--training-dir')
	parser.add_option('--round-dirs',dest='set_training_round_dirs',action='store_true')
	parser.add_option('--set-training-round-dirs',action='store_true',default=False)
	
	parser.add_option('--seed')
	parser.add_option('--random',action='store_true',default=False)
	parser.add_option('--num-samples',type='int',default=1)
	parser.add_option('--sample-all',action='store_true',default=False)
	
	parser.add_option("--num-rounds",type='int',default=0)
	parser.add_option("--one-shot",dest='train-all',action='store_true')
	parser.add_option("--validate-only",action='store_true',default=False)
	parser.add_option("--validation-only",action='store_true',dest='validate_only')
	parser.add_option("--train-only",action='store_true')
	parser.add_option('--train-all',action='store_true',default=False)
	
	parser.add_option('--dont-validate-training-images',action='store_true',default=False)
	parser.add_option('--remove-from-validation-dir')
	
	parser.add_option('--clear',action='store_true',default=False)
	parser.add_option('--clear-first',action='store_true',default=False)
	
	(options,args) = parser.parse_args()
	
	if not options.test and (options.clear or options.clear_first):
		print "clearing..."
		res = GoogleGoggles.clear()
		print res['status']
		if not options.clear_first:
			return
	
	tester = ImageTester(options,args)
	
	tester.run()
	
if __name__ == "__main__":
	main(sys.argv)