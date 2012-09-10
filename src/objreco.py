import os.path, time
import urllib, urllib2, json

class GoogleGoggles(object):

    SERVER = "http://amp.google.com/"
    LEARN = "objreco/learn"
    MATCH = "objreco/match"
    
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
    def _prepare_request(url, img_path):
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
        

class GoogleGoggles2(object):

    DEFAULT_SERVER = "http://amp.google.com/"
    DEFAULT_LEARN_ENDPOINT = "objreco/learn"
    DEFAULT_MATCH_ENDPOINT = "objreco/match"
    DEFAULT_MIN_CALL_INTERVAL = 0.5
    
    def __init__(self,**kwargs):
		self.last_call_time=None
		
		if kwargs.has_key('server'):
			self.server = kwargs['server']
		else:
			self.server = DEFAULT_SERVER
		
		if kwargs.has_key('learn'):
			self.learn = kwargs['learn']
		else:
			self.learn = DEFAULT_LEARN_ENDPOINT
		
		if kwargs.has_key('match'):
			self.match = kwargs['match']
		else:
			self.match = DEFAULT_MATCH_ENDPOINT
			
		if kwargs.has_key('min_call_interval'):
			self.min_call_interval = kwargs['match']
		else:
			self.min_call_interval = DEFAULT_MIN_CALL_INTERVAL
    
    def throttle(self):
        now = time.mktime(time.localtime())
        if not self.last_call_time:
            self.last_call_time = now
        next_call = (self.last_call_time + self.min_call_interval)
        if self.last_call_time and now < next_call:
           time.sleep(next_call-now)
        self.last_call_time = now

    @staticmethod
    def _prepare_request(url, img_path):
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
    def parse_response(res):
       return json.loads(res.replace("status","'status'").replace("image_label","'image_label'").replace("match_score","'match_score'").replace("image_id","'image_id'").replace("'",'"'))

    def learn(self,img_path, label):
        self.throttle()
        url = GoogleGoggles2._prepare_url( \
                self.server + self.learn,
                {"label": label, "serverid": 1})
        request = GoogleGoggles2._prepare_request(url, img_path)
        res = urllib2.urlopen(request).read().strip()
        return GoogleGoggles2.parse_response(res)

    def match(self,img_path):
        self.throttle()
        url = GoogleGoggles2._prepare_url( \
                self.server + self.MATCH,
                {"serverid": 1})
        request = GoogleGoggles2._prepare_request(url, img_path)
        #print "requesting..."
        #print request.header_items()
        res = urllib2.urlopen(request).read().strip()
        return GoogleGoggles2.parse_response(res)