# -*- coding: utf-8 -*-
"""
Created on Thu Jan 17 15:35:38 2013

@author: benk
"""

import roslib
roslib.load_manifest('google_goggles')

from collections import defaultdict

import sys, os, os.path, random, time, math

import numpy as np
import pandas as pd

TIME_FORMAT = "%Y-%m-%d-T%H-%M-%S"

from google_goggles_connector import GoogleGoggles

def flatten(list_of_lists):
	l = []
	t = []
	for outer_idx,lst in enumerate(list_of_lists):
		for inner_idx,item in enumerate(lst):
			l.append(item)
			t.append((outer_idx,inner_idx))
	return l,t

def flat_series(list_of_lists):
	l,t = flatten(list_of_lists)
	if l:
		index = pd.MultiIndex.from_tuples(t)
	else:
		l = [np.nan]
		index = ['EMPTY']
	return pd.Series(l,index=index)

class ImageTester(object):
	def __init__(self,options,args):
		self.objects = set()
		
		self.image_dirs = []
		self.images = []
		
		self.image_dir_series = pd.DataFrame({'dir':[],'type':[]})
		self.TRAINING_DIR = 0
		self.TESTING_DIR = 1
		self.REMOVE_FROM_TESTING_DIR = 2
		
		self.image_db = None
		self.schedule = None
		
		self.training = pd.DataFrame({})
		self.img_labels = pd.Series([],name='label')
		self.tests = pd.DataFrame({})
		self.matches = pd.DataFrame({},dtype=float)
		
		self.field_names = ['label','pose','bg','ltng','view']
		self.field_vals = [[],[],[],[],[]]
		
		self.validation_images = None
		self.training_images = None
		self.images_not_learned = None
		self.training_image_names = None
		self.images_learned = None
		
		self.data_table = None
		
		self.options = options
		
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
	
	def run(self):
		if self.options.test:
			print 'TESTING!'
	
		if not self.options.validate_only:
			if len(self.options.training_dir) == 1:
				print "Training dir: %s" % self.options.training_dir[0]
			else:
				print "Training dirs:"
				for d in self.options.training_dir:
					print "  %s" % d
		
		if not self.options.train_only:
			print "Validation dir: %s" % self.options.validation_dir
			self.image_dirs = [self.options.validation_dir]
			
		self.image_dirs = self.image_dirs + self.options.training_dir
		
		self.image_dir_series = self.image_dir_series.append({'dir':self.options.validation_dir,'type':self.TESTING_DIR},ignore_index=True)
		
		num_training_dirs = len(self.options.training_dir)
		self.image_dir_series = self.image_dir_series.append(pd.DataFrame({'dir':self.options.training_dir,'type':[self.TRAINING_DIR] * num_training_dirs}),ignore_index=True)
		
		if not self.options.train_only and self.options.remove_from_validation_dir:
			self.image_dirs = [self.options.remove_from_validation_dir] + self.image_dirs
			self.image_dir_series = self.image_dir_series.append({'dir':self.options.remove_from_validation_dir,'type':self.REMOVE_FROM_TESTING_DIR},ignore_index=True)
		
		[self.images.append([]) for image_dir in self.image_dirs]
		
		
		self._build_image_database()
		
		self._init()
		
		self._execute()
		
	
	def _build_image_database(self):
		print 'Building image database'
		for idx,image_dir in enumerate(self.image_dirs):
			if idx != 0 and self.options.validate_only: continue
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
					if fields[i+1] not in self.field_vals[i]:
						self.field_vals[i].append(fields[i+1])
				
				self.images[idx].append(tuple(fields))
				self.objects.add(fields[1])
			
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
						if fields[i+1] not in self.field_vals[i]:
							self.field_vals[i].append(fields[i+1])
					
					self.images[idx].append(tuple(fields))
					self.objects.add(fields[1])
		
		extended_field_names = ['directory'] + ['image'] + self.field_names
		image_db_dict = defaultdict(list)
		image_db_index_tuples = []
		for image_dir_idx, image_dir in enumerate(self.image_dirs):
			for image in self.images[image_dir_idx]:
				image_db_index_tuples.append((image_dir,image[0]))
				image_db_dict['directory'].append(image_dir)
				for idx,field_name in enumerate(extended_field_names[1:]):
					image_db_dict[field_name].append(image[idx])
					
		
		image_db_index = None
		#image_db_index = image_db_dict['image']
		image_db_index = pd.MultiIndex.from_tuples(image_db_index_tuples)
		#image_db_index = image_db_index_tuples
		self.image_db = pd.DataFrame(data=image_db_dict,columns=extended_field_names,index=image_db_index)
		
		self.image_db['id'] = np.nan
		
		schedule_tmp = pd.Series(data=np.nan,index=self.image_db.index)
		self.schedule = pd.DataFrame(data={'train':schedule_tmp,'test':schedule_tmp})
		
	def _init(self):
		if self.options.train_only:
			self.validation_images = []
		else:
			remove = []
			if self.options.remove_from_validation_dir:
				remove = self.images[0]
				del self.images[0]
			self.validation_images = self.images[0]
			del self.images[0]
			remove_names = [os.path.basename(img[0]) for img in remove]
			self.validation_images = [img for img in self.validation_images if os.path.basename(img[0]) not in remove_names]
		
		self.training_images = self.images
		
		#self.images = None
		
		self.images_not_learned = []
		self.images_not_learned[:] = [item for sublist in self.training_images for item in sublist]
		
		
		self.training_image_names = [os.path.basename(img[0]) for img in self.images_not_learned]
		
		if self.options.train_all and self.options.dont_validate_training_images:
			self.validation_images = [img for img in self.validation_images if os.path.basename(img[0]) not in self.training_image_names]
		
		self.images_learned = []
		
		
		if self.options.random:
			self.random_seed = time.time()
		elif self.options.seed:
			self.random_seed = self.options.seed
		else:
			self.random_seed = 500
		self.random_seed = str(self.random_seed)
		random.seed(self.random_seed)
	
	def _execute(self):
		num_training_images = len(self.images_not_learned)
		
		print 'Testing'
		round_num = 0
		
		start_time = time.time()
		
		filename_base = 'results_'
	
		if self.options.name:
			filename_base = filename_base + self.options.name + '_'
		
		filename_base = filename_base + time.strftime(TIME_FORMAT,time.localtime(start_time))
		if self.options.test:
			filename_base = 'test_' + filename_base
		
		filename = os.path.join(self.options.save_dir,filename_base + '.txt')
		
		f = open(filename,'w')
		
		err_f = None
		
		pickle_filename = os.path.join(self.options.save_dir,filename_base + '.h5')
		hdf = pd.HDFStore(pickle_filename)
		pkld = {}
		
		#pkl.dump('start_time'); pkl.dump(start_time)
		#pkl.dump('root_dir'); pkl.dump(os.path.abspath(root_dir))
		#pkl.dump('random_seed'); pkl.dump(random_seed)
		
		pkld['cmd'] = ' '.join(sys.argv)
		pkld['start_time'] = start_time
		#pkld['root_dir'] = os.path.abspath(root_dir)
		if not self.options.validate_only:
			pkld['training_dir'] = self.options.training_dir
			pkld['training_dir_abs'] = [os.path.abspath(d) for d in self.options.training_dir]
		pkld['validation_dir'] = self.options.validation_dir
		pkld['validation_dir_abs'] = os.path.abspath(self.options.validation_dir)
		pkld['random_seed'] = self.random_seed
		
		f.write('Command: %s\n' % ' '.join(sys.argv))
		
		if not self.options.validate_only:
			if len(self.training_images) == 1:
				f.write('Training dir: %s (%s)\n' % (self.options.training_dir[0], os.path.abspath(self.options.training_dir[0])))
			else:
				f.write('Training dir: %s (%s)\n' % (self.options.training_dir, [os.path.abspath(d) for d in self.options.training_dir]))
		f.write('Validation dir: %s (%s)\n' % (self.options.validation_dir, os.path.abspath(self.options.validation_dir)))
		f.write('Random seed: %s\n' % self.random_seed)
		
		if not self.options.validate_only:
			f.write('Training images:\n')
			if len(self.training_images) == 1:
				[f.write(img[0]+'\n') for img in self.training_images[0]]
			else:
				for idx,t_imgs in enumerate(self.training_images):
					f.write('Round #%d\n' % (idx+1))
					[f.write(img[0]+'\n') for img in t_imgs]
		
		f.write('Validation images:\n')
		[f.write(img[0]+'\n') for img in self.validation_images]
		
		learn_responses = []
		match_responses = []
		
		learned_images_to_this_round = []
		
		false_detects = []
		
		round_times = []
		
		try:
			while self.images_not_learned or self.options.validate_only: #and not rospy.is_shutdown():
				round_num += 1
				if self.options.num_rounds >= 1:
					print "Round #%d/%d" % (round_num,self.options.num_rounds)
					f.write('Round %d/%d:\n' % (round_num,self.options.num_rounds))
				else:
					print "Round #%d" % round_num
					f.write('Round %d:\n' % (round_num))
				round_start_time = time.time()
				
				this_round_false_detects = []
				
				if not self.options.validate_only:
					print 'Learning images...'
					learn_responses.append([])
					images_for_this_round = []
					if len(self.training_images) == 1:
						images_for_this_round[:] = self.images_not_learned
					else:
						images_for_this_round[:] = self.training_images[round_num-1]
					images_learned_this_round = []
					
					if self.options.train_all:
						print 'Learning all %d images' % len(images_for_this_round)
						imgs_to_learn = images_for_this_round
					elif self.options.sample_all:
						num_samples = min(self.options.num_samples,len(images_for_this_round))
						print 'Sampling %d images from %d across all objects' % (num_samples,len(images_for_this_round))
						imgs_to_learn = random.sample(images_for_this_round,min(self.options.num_samples,len(images_for_this_round)))
					else:
						print 'Sampling %d images for each object' % self.options.num_samples
						imgs_to_learn = []
						for object_name in sorted(self.objects):
							#if rospy.is_shutdown(): break
							#print object_name
							object_images = [img for img in images_for_this_round if img[1] == object_name]
							if not object_images: continue
							
							imgs_to_learn += random.sample(object_images,min(self.options.num_samples,len(object_images)))
					
					for img_idx,img_to_learn in enumerate(imgs_to_learn):
						print "Learning %d/%d" % (img_idx+1,len(imgs_to_learn)),img_to_learn[1:],
						if len(self.training_images) == 1:
							t_dir = self.options.training_dir[0]
						else:
							t_dir = self.options.training_dir[round_num-1]
						if not self.options.test:
							try:
								res = GoogleGoggles.learn(os.path.join(t_dir,img_to_learn[0]),img_to_learn[1])
							except Exception, e:
								print 'Exception occured during call to learn:',e
								res = {'status':'EXCEPTION','exception':e}
								if not err_f:
									err_filename = os.path.join(self.options.save_dir,filename_base + '_ERRORS.txt')
									err_f = open(err_filename,'w')
								err_f.write("LEARNING EXCEPTION [%s]: %s\n" % (os.path.join(t_dir,img_to_learn[0]),e))
						else:
							res = {'status':'SUCCESS'}

						
						learn_responses[-1].append((t_dir,img_to_learn,res))
						print res['status']
						if res['status'] != 'SUCCESS':
							#print 'failed!'
							continue
						#print 'done'
					
						image_index = (t_dir,img_to_learn[0])
						img_id = res['image_id']
						self.training = self.training.append(pd.DataFrame(
							{'status':res['status'], 'id':img_id},index=[image_index]))
						
						self.image_db.set_value(image_index,'id', img_id)
						self.schedule.set_value(image_index,'learn', round_num)
						
						self.img_labels = self.img_labels.set_value(img_id,img_to_learn[1])
						
						self.images_not_learned.remove(img_to_learn)
						images_learned_this_round.append(img_to_learn)
					
					self.images_learned.append(images_learned_this_round)
					
					f.write('Learned:\n')
					[f.write(img[0]+'\n') for img in self.images_learned[round_num-1]]
					learned_images_to_this_round = [item for sublist in self.images_learned[:round_num] for item in sublist]
					
					#learned 
					for object_name in sorted(self.objects):
						f.write('%s:\n' % object_name)
						
						for fieldidx,field_name in enumerate(self.field_names):
							if fieldidx == 0: continue
							f.write('  %s:\n' % field_name)
							learned_fields = defaultdict(int)
							for imgidx,img in enumerate(learned_images_to_this_round):
								if img[1] != object_name: continue
								field_val = img[fieldidx+1]
								learned_fields[field_val] = learned_fields[field_val] + 1
							
							for key,val in learned_fields.iteritems():
								f.write("	%s: %d\n" % (key,val))
					
					for idx,field_name in enumerate(self.field_names):
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
				data_this_round = np.zeros((len(self.validation_images),1),dtype=int)
				confidence_this_round = np.zeros((len(self.validation_images),1),dtype=float)
				
				if self.options.dont_validate_training_images:
					num_images_to_validate = len(self.validation_images) - len(learned_images_to_this_round)
				else:
					num_images_to_validate = len(self.validation_images)
				
				actually_tested_idx = -1
				for idx,img in enumerate(self.validation_images):
					#if rospy.is_shutdown(): break
					
					actually_tested_idx+=1
					#print 'testing %d/%d' % (actually_tested_idx+1,num_images_to_validate) , img[1:],
					print 'testing %d/%d' % (idx+1,len(self.validation_images)) , img[1:],
					
					if self.options.dont_validate_training_images and \
							[li for li in learned_images_to_this_round if os.path.basename(img[0]) == os.path.basename(li[0])]:
						res = {'status':'SUCCESS',
								'matches':[{'image_label':img[1]}],
								'skipped':True}
						success = True
					elif not self.options.test:
						try:
							res = GoogleGoggles.match(os.path.join(self.options.validation_dir,img[0]))
						except Exception, e:
							print 'Exception occured during call to learn:',e
							res = {'status':'EXCEPTION','exception':e,'matches':[]}
							if not err_f:
								err_filename = os.path.join(self.options.save_dir,filename_base + '_ERRORS.txt')
								err_f = open(err_filename,'w')
							err_f.write("MATCHING EXCEPTION [%s]: %s\n" % (os.path.join(self.options.validation_dir,img[0]),e))
						
					else:
						if self.data_table is not None and self.data_table[idx,-1]:
							success = True
						else:
							choice = random.random()
							if choice < 0.5:
								res = {'status':'SUCCESS',
										'matches':[{'image_label':img[1]}]}
							else:
								choice = random.random()
								if choice < 0.25:
									#false detection
									res = {'status':'SUCCESS',
											'matches':[{'image_label':random.choice([o for o in self.objects if o != img[1]])}]}
								else:
									res = {'status':'FAILURE','matches':[]}
					
					match_responses[-1].append((self.options.validation_dir,img,res))
					success = res['status'] == 'SUCCESS' and res['matches'] and res['matches'][0]['image_label'] == img[1]
					
					image_index = (self.options.validation_dir,img[0])
					
					test_index = tuple([round_num] + list(image_index))
					
					
					
					if not res.get('skipped'):
						self.tests = self.tests.append(pd.DataFrame({'image':[image_index], 'label': [img[1]], 'status':[res['status']]},index=[test_index]))
						
						self.schedule.set_value(image_index,'test', round_num)
						
						self.matches[test_index] = np.nan
					
						for match in res['matches']:
							img_id = match.get('image_id',-1)
							label = match['image_label']
							confidence = float(match.get('match_score',np.nan))
							self.matches = self.matches.set_value(img_id,test_index,confidence)
							self.img_labels = self.img_labels.set_value(img_id,label)
					
					if res.get('skipped'):
						print '[skipped]'
					elif res['matches'] and res['matches'][0]['image_label'] and not success:
						print 'FALSE DETECT:',res['matches'][0]['image_label']
						this_round_false_detects.append((img,res['matches'][0]['image_label']))
					elif success or not res['matches']:
						print res['status']
					else:
						print res['status'], res['matches'][0]['image_label']
					#print 'done', success
						
					
					data_this_round[idx,0] = int(success)
					confidence_this_round[idx,0] = int(success)
				
				num_successes_this_round = np.sum(data_this_round[:,0])
				total_this_round = len(self.validation_images)
				
				print '  %d/%d successes' % (num_successes_this_round,total_this_round)
				
				if self.data_table is None:
					self.data_table = data_this_round
				else:
					self.data_table = np.hstack((self.data_table,data_this_round))
				
				f.write('Test results:\n')
				f.write('Summary: %d/%d\n' % (np.sum(self.data_table[:,round_num-1].flatten()),len(self.validation_images)))
				for object_name in sorted(self.objects):
					successes = 0
					total = 0
					for imgidx,img in enumerate(self.validation_images):
						if img[1] == object_name:
							successes += self.data_table[imgidx,round_num-1]
							total += 1
					f.write('%s: %d/%d\n' % (object_name,successes,total))
					
					for fieldidx,field_name in enumerate(self.field_names):
						if fieldidx == 0: continue
						f.write('  %s:\n' % field_name)
						tested_fields = {}
						for imgidx,img in enumerate(self.validation_images):
							if img[1] != object_name: continue
							field_val = img[fieldidx+1]
							if not tested_fields.has_key(field_val):
								tested_fields[field_val] = [0,0]
							tested_fields[field_val][0] = tested_fields[field_val][0] + self.data_table[imgidx,round_num-1]
							tested_fields[field_val][1] = tested_fields[field_val][1] + 1
						
						for key,val in tested_fields.iteritems():
							f.write("	%s: %d/%d\n" % (key,val[0],val[1]))
				
				for fieldidx,field_name in enumerate(self.field_names):
					f.write('%s:\n' % field_name)
					tested_fields = {}
					for imgidx,img in enumerate(self.validation_images):
						field_val = img[fieldidx+1]
						if not tested_fields.has_key(field_val):
							tested_fields[field_val] = [0,0]
						tested_fields[field_val][0] = tested_fields[field_val][0] + self.data_table[imgidx,round_num-1]
						tested_fields[field_val][1] = tested_fields[field_val][1] + 1
					
					for key,val in tested_fields.iteritems():
						f.write("  %s: %d/%d\n" % (key,val[0],val[1]))
			
				if this_round_false_detects:
					f.write('False detects:\n')
					for img,label in this_round_false_detects:
						f.write('%s: %s\n' % (img[0],label))
				else:
					f.write('No false detects\n')
				false_detects.append(this_round_false_detects)
				
				f.write('Raw data:\n')
				f.write(str(self.data_table[:,round_num-1].flatten().tolist())+'\n')
				
				round_end_time = time.time()
				round_time = (round_end_time - round_start_time)
				print 'Round complete, took %f seconds' % round_time
				f.write('Round time: %fs\n' % round_time)
				round_times.append(round_time)
				
				if (not self.options.train_only and np.all(self.data_table[:,-1])) \
						or round_num == self.options.num_rounds \
						or not self.images_not_learned:
					break
		except Exception, e:
			print 'Exception occurred:',e
			f.write('Exception occurred: %s\n' % str(e))
			#pkl.dump(e)
			if not err_f:
				err_filename = os.path.join(self.options.save_dir,filename_base + '_ERRORS.txt')
				err_f = open(err_filename,'w')
			err_f.write("LOOP EXCEPTION: %s\n" % e)
			raise e
			
		
		num_rounds = len(self.images_learned)
		all_learned_images = [item for sublist in self.images_learned for item in sublist]
		
		end_time = time.time()
		total_time = end_time - start_time
		total_minutes = math.floor(total_time/60)
		total_seconds_into_minute = total_time - 60 * total_minutes
		
		if not self.options.validate_only:
			pkld['training_images'] = self.training_images
		pkld['validation_images'] = self.validation_images
		pkld['field_names'] = self.field_names
		pkld['field_vals'] = self.field_vals
		pkld['objects'] = self.objects
		if not self.options.validate_only:
			pkld['images_learned'] = self.images_learned
			pkld['images_not_learned'] = self.images_not_learned
		pkld['num_rounds'] = num_rounds
		
		if not self.options.validate_only:
			pkld['learn_responses'] = learn_responses
		pkld['match_responses'] = match_responses
		
		pkld['false_detects'] = false_detects
		#pkld['num_false_detects'] = num_false_detects
		
		#pf = open(pickle_filename,'w')
		#pickle.dump(pkld,pf)
		#pf.close()
		
		print 'Complete'
		f.write('Complete\n')
		
		
		print 'Total time: %d minutes, %f seconds' % (total_minutes,total_seconds_into_minute)
		f.write('Total time: %d minutes, %f seconds\n' % (total_minutes,total_seconds_into_minute))
		
		print 'Total learn calling time: %f seconds' % (GoogleGoggles.NUM_LEARN_CALLS * GoogleGoggles.AVG_LEARN_CALL_TIME)
		f.write('Total learn calling time: %f seconds\n' % (GoogleGoggles.NUM_LEARN_CALLS * GoogleGoggles.AVG_LEARN_CALL_TIME))
		print 'Average learn calling time: %f seconds' % GoogleGoggles.AVG_LEARN_CALL_TIME
		f.write('Average learn calling time: %f seconds\n' % GoogleGoggles.AVG_LEARN_CALL_TIME)
		
		print 'Total match calling time: %f seconds' % (GoogleGoggles.NUM_MATCH_CALLS * GoogleGoggles.AVG_MATCH_CALL_TIME)
		f.write('Total match calling time: %f seconds\n' % (GoogleGoggles.NUM_MATCH_CALLS * GoogleGoggles.AVG_MATCH_CALL_TIME))
		print 'Average match calling time: %f seconds' % GoogleGoggles.AVG_MATCH_CALL_TIME
		f.write('Average match calling time: %f seconds\n' % GoogleGoggles.AVG_MATCH_CALL_TIME)
		
		if not self.options.validate_only:
			print "Learned %d/%d images in %d steps" % (len(all_learned_images),num_training_images,num_rounds)
			f.write("Learned %d/%d images in %d steps\n" % (len(all_learned_images),num_training_images,num_rounds))
			for i in xrange(num_rounds):
				print "  Round #%d Learned %d" % (i+1,len(self.images_learned[i]))
				f.write("  Round #%d Learned %d\n" % (i+1,len(self.images_learned[i])))
			
			print "Training images left unlearned: %d" % len(self.images_not_learned)
			f.write("Training images left unlearned: %d images\n" % len(self.images_not_learned))
		
		for i in xrange(num_rounds):
			if i >= self.data_table.shape[1]:
				print '  Round #%d: Data doesn\'t exist' % (i+1)
				f.write('  Round #%d: Data doesn\'t exist' % (i+1))
			else:
				print '  Round #%d Recognized %d/%d' % (i+1,np.sum(self.data_table[:,i].flatten()),len(self.validation_images))
				f.write('  Round #%d Recognized %d/%d\n' % (i+1,np.sum(self.data_table[:,i].flatten()),len(self.validation_images)))
		
		all_false_detects = [item for sublist in false_detects for item in sublist]
		if all_false_detects:
			print 'False detects: %d' % len(all_false_detects)
			f.write('False detects: %d\n' % len(all_false_detects))
			for img,label in all_false_detects:
				print '%s: %s' % (img[0],label)
				f.write('%s: %s\n' % (img[0],label))
		else:
			print 'No false detects'
			f.write('No false detects\n')
		
		
		print "Wrote results to %s" % filename
		
		try:
			hdf.put('image_dirs', self.image_dir_series)
			hdf.put('dir_types', pd.Series({'training':self.TRAINING_DIR,'testing':self.TESTING_DIR,'remove_from_testing':self.REMOVE_FROM_TESTING_DIR}))
		
			hdf.put('image_db', self.image_db)
			hdf.put('schedule', self.schedule)
		
			self.training.index = pd.MultiIndex.from_tuples(self.training.index)
			hdf.put('training', self.training)
			hdf.put('image_labels', self.img_labels)
		
			self.tests.index = pd.MultiIndex.from_tuples(self.tests.index)
			hdf.put('tests', self.tests)
		
			self.matches.columns = pd.MultiIndex.from_tuples(self.matches.columns)
			if np.size(self.matches):
				hdf.put('matches', self.matches)

			hdf.put('info/cmd', pd.Series([' '.join(sys.argv)]))
			
			hdf.put('info/times/overall', pd.Series([start_time,end_time,total_time],index=['start time','end time','total_time']))
			hdf.put('info/times/rounds', pd.Series(round_times))
			
			call_times = {}
			call_times['learn_total'] = GoogleGoggles.NUM_LEARN_CALLS * GoogleGoggles.AVG_LEARN_CALL_TIME
			call_times['learn_avg'] = GoogleGoggles.AVG_LEARN_CALL_TIME
			call_times['match_total'] = GoogleGoggles.NUM_MATCH_CALLS * GoogleGoggles.AVG_MATCH_CALL_TIME
			call_times['match_avg'] = GoogleGoggles.AVG_MATCH_CALL_TIME
			
			hdf.put('info/times/call',pd.Series(call_times))
			
			#pkld['root_dir', os.path.abspath(root_dir)
			if not self.options.validate_only:
				hdf.put('info/dirs/training', pd.Series(self.options.training_dir))
				hdf.put('info/dirs/training_abs', pd.Series([os.path.abspath(d) for d in self.options.training_dir]))
			
			if not self.options.train_only:
				hdf.put('info/dirs/validation', pd.Series([self.options.validation_dir]))
				hdf.put('info/dirs/validation_abs', pd.Series([os.path.abspath(self.options.validation_dir)]))
	
			hdf.put('info/random_seed', pd.Series([self.random_seed]))
			
			if not self.options.validate_only:
				hdf.put('images/training', flat_series(self.training_images))
			if not self.options.train_only:
				hdf.put('images/validation', flat_series(self.validation_images))
			
			hdf.put('info/fields/names', pd.Series(self.field_names))
			hdf.put('info/fields/values', pd.Series(self.field_vals))
			hdf.put('info/objects', pd.Series(list(self.objects)))
			if not self.options.validate_only:
				hdf.put('images/training/learned', flat_series(self.images_learned))
				hdf.put('images/training/not_learned', flat_series(self.images_not_learned))
			
			hdf.put('num_rounds', pd.Series([num_rounds]))
		
			if not self.options.validate_only:
				hdf.put('responses/learn', flat_series(learn_responses))
			if not self.options.train_only:
				hdf.put('responses/match', flat_series(match_responses))
		
			hdf.put('false_detects', flat_series(false_detects))

			#hdf.put('info', pd.Series(pkld))
			
			print "Wrote data to %s" % pickle_filename
		except Exception, e:
			print "Exception occurred while writing HDF: ", e
			raise e
