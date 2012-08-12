#!/usr/bin/env python
import roslib
roslib.load_manifest('tf')
from math import sqrt, isnan
import sys
from numpy import *
import numpy

from subprocess import call

files = ["fresh1.pcd", "screwdriver1.pcd",  "screwdriver5.pcd",  "soapv1.pcd",     "starburst3.pcd", "tape3.pcd",
"fresh2.pcd", "screwdriver2.pcd", "screwdriver6.pcd", "soapv2.pcd",     "starburst4.pcd", "tape4.pcd",
"juice1.pcd", "peanut1.pcd",  "screwdriver3.pcd", "soap1.pcd",        "starburst1.pcd", "tape1.pcd",
"juice2.pcd", "peanut2.pcd",  "screwdriver4.pcd", "soap2.pcd",        "starburst2.pcd", "tape2.pcd"]

files = ["screwdriver1.pcd", "screwdriver2.pcd", "screwdriver3.pcd", "screwdriver4.pcd", "screwdriver5.pcd", "screwdriver6.pcd",
"starburst1.pcd","starburst2.pcd","starburst3.pcd","starburst4.pcd"]

flip = True
if sys.argv[1] == '-u':
	flip = False
	del sys.argv[1]

files = sys.argv[1:]

maxDistFromOrigin = 0.2;
minHeight = 0.005;

for fname in files:
	print "converting file %s" % fname
	
	fname_out = fname[0:len(fname)-4] + "_adjusted.pcd"

	f = open(fname);
	fout = open(fname_out,"w");

	cnt = 0;
	kept = 0;
	reject = 0;

	minx = 1000000;
	maxx = -1000000;
	miny = 1000000;
	maxy = -1000000;
	minz = 1000000;
	maxz = -1000000;

	headerLines = []
	lines = []
	
	M = empty((10000,4));

	while True:
		l = f.readline();
		if l == "":
			break
		l = l[0:len(l)-1]
		cnt += 1
		if cnt <= 11:
			pass #fout.write(l + "\n")
		else:
			vals = l.split(" ")
			x,y,z,c = (float(vals[0]),float(vals[1]),float(vals[2]),float(vals[3]))
			
			if x < minx: minx = x
			if x > maxx: maxx = x
			if y < miny: miny = y
			if y > maxy: maxy = y
			if z < minz: minz = z
			if z > maxz: maxz = z
			
			keep = True
			#keep = keep and not isnan(x)
			#keep = keep and sqrt(x*x + y*y) < maxDistFromOrigin
			#keep = keep and z <= -minHeight
			#keep = keep and z <= 0
			
			#l = vals[0] + " " + vals[1] + " " + str(-z) + " " + vals[3]
			
			if keep:
				lines.append(l + "\n")
				kept += 1
				M[kept-1,:] = array([x,y,z,c])
			else:
				reject += 1

	M = M[0:kept,:]
	print "round 1: keep {0}, rejected {1}".format(kept,reject)
	
	center = mean(M,0)
	M[:,0:2] = M[:,0:2] - center[0:2]
	
	#print numpy.max(M,0), numpy.min(M,0)
	
	M2 = empty_like(M)
	kept2 = 0; reject2 = 0
	for i in range(kept):
		keep = True
		#if M[i,2] < 0: keep = False
		if sqrt(M[i,0]**2 + M[i,1]**2) > 0.08: keep = False
		#if M[i,1] > 0.05: keep = False
		if keep:
			kept2 += 1
			M2[kept2 - 1,:] = M[i,:]
		else:
			reject2 += 1
	
	M2 = M2[0:kept2,:]
	print "round 2: kept {0}, rejected {1}".format(kept2,reject2)
	
	center = mean(M2,0)
	M2[:,0:2] = M2[:,0:2] - center[0:2]
	
	kept_final = kept2
	M_final = M2
	
	header = "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {0}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {0}\nDATA ascii\n".format(kept_final)
	fout.write(header)
	
	for i in range(kept_final):
		x,y,z,c = (M_final[i,0],-M_final[i,1],M_final[i,2],M_final[i,3])
		if flip:
			z = -z
		fout.write("{0!r} {1!r} {2!r} {3!r}\n".format(x,y,z,c))

	#for line in lines: fout.write(line)

	#print "x: %f %f" % (minx,maxx)
	#print "y: %f %f" % (miny,maxy)
	#print "z: %f %f" % (minz,maxz)

	print "final  : kept {0}, rejected {1}".format(kept_final,len(lines) - kept_final)
	
	f.close()
	fout.close()
