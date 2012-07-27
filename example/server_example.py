import urllib, urllib2, os

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
        res = urllib2.urlopen(request).read().strip()
        return res

if __name__ == "__main__":
    print "learning kitten..."
    print GoogleGogglesConnector.learn("kitten.png", "kitten")
    print "learning beer..."
    print GoogleGogglesConnector.learn("beer.png", "beer")
    print "matching kitten..."
    print GoogleGogglesConnector.match("kitten.png")
    print "matching beer..."
    print GoogleGogglesConnector.match("beer.png")
