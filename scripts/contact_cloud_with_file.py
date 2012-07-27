#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os
import urllib, urllib2

class GoogleGogglesConnector(object):

    SERVER = "http://amp.google.com/"
    LEARN = "objreco/learn"
    MATCH = "objreco/match"

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
    def learn(img_path, label):
        url = GoogleGogglesConnector._prepare_url( \
                GoogleGogglesConnector.SERVER + GoogleGogglesConnector.LEARN,
                {"label": label, "serverid": 1})
        request = GoogleGogglesConnector._prepare_request(url, img_path)
        res = urllib2.urlopen(request).read().strip()
        return res

    @staticmethod
    def match(img_path):
        url = GoogleGogglesConnector._prepare_url( \
                GoogleGogglesConnector.SERVER + GoogleGogglesConnector.MATCH,
                {"serverid": 1})
        request = GoogleGogglesConnector._prepare_request(url, img_path)
        print "requesting..."
        print request.header_items()
        res = urllib2.urlopen(request).read().strip()
        return res

if __name__ == "__main__":
    print GoogleGogglesConnector.match(sys.argv[1])
