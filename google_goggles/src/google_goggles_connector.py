# -*- coding: utf-8 -*-
"""
Created on Thu Jan 17 15:37:42 2013

@author: benk
"""

import os.path, time
import urllib, urllib2, json

class GoogleGoggles(object):

	SERVER = "http://amp.google.com/"
	LEARN = "objreco/learn"
	MATCH = "objreco/match"
	CLEAR = "objreco/clear"
	
	NUM_LEARN_CALLS=0
	AVG_LEARN_CALL_TIME=0
	
	NUM_MATCH_CALLS=0
	AVG_MATCH_CALL_TIME=0
	
	
	LAST_CALL_TIME=None
	MIN_CALL_INTERVAL=0.5
	@staticmethod
	def _throttle():
		now = time.time()
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
		#res = res.replace("status","'status'").replace("image_label","'image_label'").replace("match_score","'match_score'").replace("image_id","'image_id'")
		res = res.replace("'",'"')
		#print res
		return json.loads(res)

	@staticmethod
	def learn(img_path, label):
		GoogleGoggles._throttle()
		tic = time.time()
		url = GoogleGoggles._prepare_url( \
				GoogleGoggles.SERVER + GoogleGoggles.LEARN,
				{"label": label, "serverid": 1})
		request = GoogleGoggles._prepare_request(url, img_path)
		res = urllib2.urlopen(request).read().strip()
		toc = time.time()
		GoogleGoggles.AVG_LEARN_CALL_TIME = ((toc-tic) + GoogleGoggles.NUM_LEARN_CALLS * GoogleGoggles.AVG_LEARN_CALL_TIME) / (GoogleGoggles.NUM_LEARN_CALLS + 1)
		GoogleGoggles.NUM_LEARN_CALLS += 1
		return GoogleGoggles.parseResponse(res)

	@staticmethod
	def match(img_path):
		GoogleGoggles._throttle()
		tic = time.time()
		url = GoogleGoggles._prepare_url( \
				GoogleGoggles.SERVER + GoogleGoggles.MATCH,
				{"serverid": 1})
		request = GoogleGoggles._prepare_request(url, img_path)
		#print "requesting..."
		#print request.header_items()
		res = urllib2.urlopen(request).read().strip()
		toc = time.time()
		GoogleGoggles.AVG_MATCH_CALL_TIME = ((toc-tic) + GoogleGoggles.NUM_MATCH_CALLS * GoogleGoggles.AVG_MATCH_CALL_TIME) / (GoogleGoggles.NUM_MATCH_CALLS + 1)
		GoogleGoggles.NUM_MATCH_CALLS += 1
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
  