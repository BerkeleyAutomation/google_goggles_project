#!/usr/bin/env python

from collections import defaultdict

import sys, os, os.path, random, time
from optparse import OptionParser
import pickle

import numpy

TIME_FORMAT = "%Y-%m-%d-T%H-%M-%S"

import os.path, time
import urllib, urllib2, json

class GoogleGoggles(object):

	SERVER = "http://amp.google.com/"
	LEARN = "objreco/learn"
	MATCH = "objreco/match"
	CLEAR = "objreco/clear"
	
	LAST_CALL_TIME=None
	MIN_CALL_INTERVAL=0.5
	@staticmethod
	def throttle():
		now = time.mktime(time.localtime())
		if not GoogleGoggles.LAST_CALL_TIME:
			GoogleGoggles.LAST_CALL_TIME = now
		next_call = (GoogleGoggles.LAST_CALL_TIME + GoogleGoggles.MIN_CALL_INTERVAL)
		if GoogleGoggles.LAST_CALL_TIME and now < next_call:
		   time.sleep(next_call-now)
		GoogleGoggles.LAST_CALL_TIME = now

	@staticmethod
	def _prepare_request(url, img_path=None):
		img_data=None
		length=0
		if img_path:
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
	def parseResponse(res):
	   return json.loads(res.replace("status","'status'").replace("image_label","'image_label'").replace("match_score","'match_score'").replace("image_id","'image_id'").replace("'",'"'))

	@staticmethod
	def learn(img_path, label):
		GoogleGoggles.throttle()
		url = GoogleGoggles._prepare_url( \
				GoogleGoggles.SERVER + GoogleGoggles.LEARN,
				{"label": label, "serverid": 1})
		request = GoogleGoggles._prepare_request(url, img_path)
		res = urllib2.urlopen(request).read().strip()
		return GoogleGoggles.parseResponse(res)

	@staticmethod
	def match(img_path):
		GoogleGoggles.throttle()
		url = GoogleGoggles._prepare_url( \
				GoogleGoggles.SERVER + GoogleGoggles.MATCH,
				{"serverid": 1})
		request = GoogleGoggles._prepare_request(url, img_path)
		#print "requesting..."
		#print request.header_items()
		res = urllib2.urlopen(request).read().strip()
		return GoogleGoggles.parseResponse(res)
	
	@staticmethod
	def clear():
		url = GoogleGoggles._prepare_url( \
				GoogleGoggles.SERVER + GoogleGoggles.CLEAR,
				{"serverid": 1})
		request = GoogleGoggles._prepare_request(url)
		#print "requesting..."
		#print request.header_items()
		res = urllib2.urlopen(request).read().strip()
		return GoogleGoggles.parseResponse(res)

def main(argv):
	parser = OptionParser('%prog [OPTIONS]')
	parser.add_option('--test',action='store_true',default=False)
	
	parser.add_option('-d','--dir',dest='validation_dir')
	parser.add_option('-v','--validation-dir',default='.')
	parser.add_option('-t','--training-dir')
	parser.add_option('--set-training-round-dirs',action='store_true',default=False)
	
	parser.add_option('--seed')
	parser.add_option('--random',action='store_true',default=False)
	parser.add_option('--num-samples',type='int',default=1)
	
	parser.add_option("--num-rounds",type='int',default=0)
	parser.add_option("--one-shot",dest='train-all',action='store_true')
	parser.add_option("--validate-only",action='store_true',default=False)
	parser.add_option("--validation-only",action='store_true',dest='validate_only')
	parser.add_option('--train-all',action='store_true',default=False)
	
	parser.add_option('--dont-validate-training-images',action='store_true',default=False)
	
	parser.add_option('--clear',action='store_true',default=False)
	parser.add_option('--clear-first',action='store_true',default=False)
	
	(options,args) = parser.parse_args()
	
	if options.clear or options.clear_first:
		print "clearing..."
		res = GoogleGoggles.clear()
		print res['status']
		if not options.clear_first:
			return
	
	if not options.training_dir:
		options.training_dir = options.validation_dir
		
	if options.validate_only:
		options.num_rounds = 1
	
	if options.set_training_round_dirs:
		if args:
			options.training_dir = args
		else:
			options.training_dir = sorted([os.path.join(options.training_dir,d) for d in os.listdir(options.training_dir) if os.path.isdir(os.path.join(options.training_dir,d))])
		options.num_rounds = len(options.training_dir)
		options.train_all = True
	else:
		options.training_dir = [options.training_dir]
		
	if options.test:
		print 'TESTING!'
	
	objects = set()
	
	if len(options.training_dir) == 1:
		print "Training dir: %s" % options.training_dir[0]
	else:
		print "Training dirs:"
		for d in options.training_dir:
			print "  %s" % d
	
	print "Validation dir: %s" % options.validation_dir
	
	image_dirs = [options.validation_dir] + options.training_dir
	images = []
	[images.append([]) for image_dir in image_dirs]
	
	field_names = ['object','pose','bg','ltng','view']
	field_vals = [[],[],[],[],[]]
	
	print 'Building image database'
	for idx,image_dir in enumerate(image_dirs):
		if idx != 0 and options.validate_only: continue
		recurse = True
		for f in os.listdir(image_dir):
			if not os.path.isdir(os.path.join(image_dir,f)) and f.endswith('.jpg'):
				recurse = False
				break
		
		files = []
		if recurse:
			for object_name in os.listdir(image_dir):
				if not os.path.isdir(os.path.join(image_dir,object_name)): continue
				
				for f in sorted(os.listdir(os.path.join(image_dir,object_name))):
					if (not os.path.isdir(f)) and f.endswith('.jpg'):
						files.append(os.path.join(object_name,f))
		else:
			for f in sorted(os.listdir(image_dir)):
				if not os.path.isdir(f) and f.endswith('.jpg'):
					files.append(f)
		
		for image in files:
			fields = [image] + os.path.basename(image).split('_')[0:5]
			if fields[-1].startswith('20'):
				fields[-1] = 'low'
			
			for i in xrange(0,5):
				if fields[i+1] not in field_vals[i]:
					field_vals[i].append(fields[i+1])
			
			images[idx].append(tuple(fields))
			objects.add(fields[1])
		
		for object_name in os.listdir(image_dir):
			break
			if not os.path.isdir(object_name): continue
			
			for image in sorted(os.listdir(os.path.join(image_dir,object_name))):
				if os.path.isdir(image): continue
				if image[-4:] != '.jpg': continue
				
				fields = [os.path.join(object_name,image)] + os.path.basename(image).split('_')[0:5]
				if fields[-1].startswith('20'):
					fields[-1] = 'low'
				
				for i in xrange(0,5):
					if fields[i+1] not in field_vals[i]:
						field_vals[i].append(fields[i+1])
				
				images[idx].append(tuple(fields))
				objects.add(fields[1])
	
	training_images = images[1:]
	validation_images = images[0]
	images = None
	
	images_not_learned = []
	images_not_learned[:] = [item for sublist in training_images for item in sublist]
	num_training_images = len(images_not_learned)
	
	training_image_names = [os.path.basename(img[0]) for img in images_not_learned]
	
	if options.train_all and options.dont_validate_training_images:
		validation_images = [img for img in validation_images if os.path.basename(img[0]) not in training_image_names]
	
	images_learned = []
	
	data_table = None
	
	print 'Testing'
	round_num = 0
	
	if options.random:
		random_seed = time.mktime(time.localtime())
	elif options.seed:
		random_seed = options.seed
	else:
		random_seed = 500
	random_seed = str(random_seed)
	random.seed(random_seed)
	
	start_time = time.localtime()
	
	filename_base = 'results_' + time.strftime(TIME_FORMAT,start_time)
	if options.test:
		filename_base = 'test_' + filename_base
	
	filename = filename_base + '.txt'
	
	f = open(filename,'w')
	
	pickle_filename = filename_base + '.pkl'
	pkld = {}
	
	#pkl.dump('start_time'); pkl.dump(start_time)
	#pkl.dump('root_dir'); pkl.dump(os.path.abspath(root_dir))
	#pkl.dump('random_seed'); pkl.dump(random_seed)
	
	pkld['start_time'] = start_time
	#pkld['root_dir'] = os.path.abspath(root_dir)
	if not options.validate_only:
		pkld['training_dir'] = options.training_dir
		pkld['training_dir_abs'] = [os.path.abspath(d) for d in options.training_dir]
	pkld['validation_dir'] = options.validation_dir
	pkld['validation_dir_abs'] = os.path.abspath(options.validation_dir)
	pkld['random_seed'] = random_seed
	
	if not options.validate_only:
		if len(training_images) == 1:
			f.write('Training dir: %s (%s)\n' % (options.training_dir[0], os.path.abspath(options.training_dir[0])))
		else:
			f.write('Training dir: %s (%s)\n' % (options.training_dir, [os.path.abspath(d) for d in options.training_dir]))
	f.write('Validation dir: %s (%s)\n' % (options.validation_dir, os.path.abspath(options.validation_dir)))
	f.write('Random seed: %s\n' % random_seed)
	
	if not options.validate_only:
		f.write('Training images:\n')
		if len(training_images) == 1:
			[f.write(img[0]+'\n') for img in training_images[0]]
		else:
			for idx,t_imgs in enumerate(training_images):
				f.write('Round #%d\n' % (idx+1))
				[f.write(img[0]+'\n') for img in t_imgs]
	
	f.write('Validation images:\n')
	[f.write(img[0]+'\n') for img in validation_images]
	
	learn_responses = []
	match_responses = []
	
	learned_images_to_this_round = []
	
	try:
		while images_not_learned or options.validate_only: #and not rospy.is_shutdown():
			round_num += 1
			print "Round #%d" % round_num
			f.write('Round %d:\n' % (round_num))
			round_start_time = time.mktime(time.localtime())
			
			if not options.validate_only:
				print 'Learning images...'
				learn_responses.append([])
				if len(training_images) == 1:
					images_for_this_round = images_not_learned
				else:
					images_for_this_round = training_images[round_num-1]
				images_learned_this_round = []
				for object_name in sorted(objects):
					#if rospy.is_shutdown(): break
					#print object_name
					object_images = [img for img in images_for_this_round if img[1] == object_name]
					if not object_images: continue
					
					if options.train_all:
						imgs_to_learn = object_images
					else:
						imgs_to_learn = random.sample(object_images,options.num_samples)
					
					for img_to_learn in imgs_to_learn:
						print "Learning",img_to_learn[1:],
						if len(training_images) == 1:
							t_dir = options.training_dir[0]
						else:
							t_dir = options.training_dir[round_num-1]
						if not options.test:
							try:
								res = GoogleGoggles.learn(os.path.join(t_dir,img_to_learn[0]),object_name)
							except Exception, e:
								print 'Exception occured during call to learn:',e
								res = {'status':'EXCEPTION','exception':e}
						else:
							res = {'status':'SUCCESS'}
						
						learn_responses[-1].append((t_dir,img_to_learn,res))
						print res['status']
						if res['status'] != 'SUCCESS':
							#print 'failed!'
							print res
							continue
						#print 'done'
					
						images_not_learned.remove(img_to_learn)
						images_learned_this_round.append(img_to_learn)
				
				images_learned.append(images_learned_this_round)
				
				f.write('Learned:\n')
				[f.write(img[0]+'\n') for img in images_learned[round_num-1]]
				learned_images_to_this_round = [item for sublist in images_learned[:round_num] for item in sublist]
				
				#learned 
				for object_name in sorted(objects):
					f.write('%s:\n' % object_name)
					
					for fieldidx,field_name in enumerate(field_names):
						if fieldidx == 0: continue
						f.write('  %s:\n' % field_name)
						learned_fields = defaultdict(int)
						for imgidx,img in enumerate(learned_images_to_this_round):
							if img[1] != object_name: continue
							field_val = img[fieldidx+1]
							learned_fields[field_val] = learned_fields[field_val] + 1
						
						for key,val in learned_fields.iteritems():
							f.write("	%s: %d\n" % (key,val))
				
				for idx,field_name in enumerate(field_names):
					if idx == 0: continue
					f.write('%s:\n' % field_name)
					learned_fields = defaultdict(int)
					for img in learned_images_to_this_round:
						field_val = img[idx+1]
						learned_fields[field_val] = learned_fields[field_val] + 1
					
					for key,val in learned_fields.iteritems():
						f.write("  %s: %d\n" % (key,val))
			
			print 'Testing images...'
			match_responses.append([])
			data_this_round = numpy.zeros((len(validation_images),1),dtype=int)
			for idx,img in enumerate(validation_images):
				#if rospy.is_shutdown(): break
				
				print 'testing %d/%d' % (idx+1,len(validation_images)) , img[1:],
				if options.dont_validate_training_images and \
						[li for li in learned_images_to_this_round if os.path.basename(img[0]) == os.path.basename(li[0])]:
					res = {'status':'SUCCESS','image_label':img[1],'skipped':True}
					success = True
				elif not options.test:
					try:
						res = GoogleGoggles.match(os.path.join(options.validation_dir,img[0]))
					except Exception, e:
						print 'Exception occured during call to learn:',e
						res = {'status':'EXCEPTION','exception':e,'image_label':''}
					success = res['image_label'] == img[1]
				else:
					if data_table is not None and data_table[idx,-1]:
						success = True
					else:
						success = bool(random.randint(0,1))
					if success:
						res = {'status':'SUCCESS','image_label':img[1]}
					else:
						res = {'status':'FAILURE','image_label':''}
				match_responses[-1].append((options.validation_dir,img,res))
				if success:
					print res['status']
				else:
					print res['status'], res['image_label']
				#print 'done', success
					
				
				data_this_round[idx,0] = int(success)
			
			print '  %d/%d successes' % (numpy.sum(data_this_round[:,0]),len(validation_images))
			
			if data_table is None:
				data_table = data_this_round
			else:
				data_table = numpy.hstack((data_table,data_this_round))
			
			f.write('Test results:\n')
			f.write('Summary: %d/%d\n' % (numpy.sum(data_table[:,round_num-1].flatten()),len(validation_images)))
			for object_name in sorted(objects):
				successes = 0
				total = 0
				for imgidx,img in enumerate(validation_images):
					if img[1] == object_name:
						successes += data_table[imgidx,round_num-1]
						total += 1
				f.write('%s: %d/%d\n' % (object_name,successes,total))
				
				for fieldidx,field_name in enumerate(field_names):
					if fieldidx == 0: continue
					f.write('  %s:\n' % field_name)
					tested_fields = {}
					for imgidx,img in enumerate(validation_images):
						if img[1] != object_name: continue
						field_val = img[fieldidx+1]
						if not tested_fields.has_key(field_val):
							tested_fields[field_val] = [0,0]
						tested_fields[field_val][0] = tested_fields[field_val][0] + data_table[imgidx,round_num-1]
						tested_fields[field_val][1] = tested_fields[field_val][1] + 1
					
					for key,val in tested_fields.iteritems():
						f.write("	%s: %d/%d\n" % (key,val[0],val[1]))
			
			for fieldidx,field_name in enumerate(field_names):
				f.write('%s:\n' % field_name)
				tested_fields = {}
				for imgidx,img in enumerate(validation_images):
					field_val = img[fieldidx+1]
					if not tested_fields.has_key(field_val):
						tested_fields[field_val] = [0,0]
					tested_fields[field_val][0] = tested_fields[field_val][0] + data_table[imgidx,round_num-1]
					tested_fields[field_val][1] = tested_fields[field_val][1] + 1
				
				for key,val in tested_fields.iteritems():
					f.write("  %s: %d/%d\n" % (key,val[0],val[1]))
		
			f.write('Raw data:\n')
			f.write(str(data_table[:,round_num-1].flatten().tolist())+'\n')
			
			round_end_time = time.mktime(time.localtime())
			round_time = (round_end_time - round_start_time)
			print 'Round complete, took %f seconds' % round_time
			f.write('Round time: %fs\n' % round_time)
			
			if numpy.all(data_table[:,-1]) \
					or round_num == options.num_rounds \
					or not images_not_learned:
				break
	except Exception, e:
		print 'Exception occurred:',e
		f.write('Exception occurred: %s\n' % str(e))
		#pkl.dump(e)
	
	num_rounds = len(images_learned)
	all_learned_images = [item for sublist in images_learned for item in sublist]
	
	end_time = time.mktime(time.localtime())
	total_time = end_time - time.mktime(start_time)
	total_minutes = round(total_time/60)
	total_seconds_into_minute = total_time - 60 * total_minutes
	
	if not options.validate_only:
		pkld['training_images'] = training_images
	pkld['validation_images'] = validation_images
	pkld['field_names'] = field_names
	pkld['field_vals'] = field_vals
	pkld['objects'] = objects
	if not options.validate_only:
		pkld['images_learned'] = images_learned
		pkld['images_not_learned'] = images_not_learned
	pkld['num_rounds'] = num_rounds
	pkld['data_table'] = data_table
	
	if not options.validate_only:
		pkld['learn_responses'] = learn_responses
	pkld['match_responses'] = match_responses
	
	pf = open(pickle_filename,'w')
	pickle.dump(pkld,pf)
	pf.close()
	
	print 'Complete'
	f.write('Complete\n')
	
	print 'Total time: %d minutes, %f seconds' % (total_minutes,total_seconds_into_minute)
	f.write('Total time: %d minutes, %f seconds\n' % (total_minutes,total_seconds_into_minute))
	
	if not options.validate_only:
		print "Learned %d/%d images in %d steps" % (len(all_learned_images),num_training_images,num_rounds)
		f.write("Learned %d/%d images in %d steps\n" % (len(all_learned_images),num_training_images,num_rounds))
		for i in xrange(num_rounds):
			print "  Round #%d Learned %d" % (i+1,len(images_learned[i]))
			f.write("  Round #%d Learned %d\n" % (i+1,len(images_learned[i])))
		
		print "Training images left unlearned: %d" % len(images_not_learned)
		f.write("Training images left unlearned: %d images\n" % len(images_not_learned))
	
	for i in xrange(num_rounds):
		if i >= data_table.shape[1]:
			print '  Round #%d: Data doesn\'t exist' % (i+1)
			f.write('  Round #%d: Data doesn\'t exist' % (i+1))
		else:
			print '  Round #%d Recognized %d/%d' % (i+1,numpy.sum(data_table[:,i].flatten()),len(validation_images))
			f.write('  Round #%d Recognized %d/%d\n' % (i+1,numpy.sum(data_table[:,i].flatten()),len(validation_images)))
	
	f.close()
	
	print "Wrote results to %s" % filename
	
if __name__ == "__main__":
	main(sys.argv)