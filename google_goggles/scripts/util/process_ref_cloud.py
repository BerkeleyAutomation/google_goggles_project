#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import sqrt, isnan
import sys

from subprocess import call

files = ["fresh1.pcd", "screwdriver1.pcd",  "screwdriver5.pcd",  "soapv1.pcd",     "starburst3.pcd", "tape3.pcd",
"fresh2.pcd", "screwdriver2.pcd", "screwdriver6.pcd", "soapv2.pcd",     "starburst4.pcd", "tape4.pcd",
"juice1.pcd", "peanut1.pcd",  "screwdriver3.pcd", "soap1.pcd",        "starburst1.pcd", "tape1.pcd",
"juice2.pcd", "peanut2.pcd",  "screwdriver4.pcd", "soap2.pcd",        "starburst2.pcd", "tape2.pcd"]

files = ["screwdriver1.pcd", "screwdriver2.pcd", "screwdriver3.pcd", "screwdriver4.pcd", "screwdriver5.pcd", "screwdriver6.pcd",
"starburst1.pcd","starburst2.pcd","starburst3.pcd","starburst4.pcd"]

maxDistFromOrigin = 0.2;
minHeight = 0.005;

for fname in files:
	print "converting file %s" % fname
	
	remote_file = "pr2base.eecs.berkeley.edu:/home/benk/clouds/"+fname
	call(["scp", remote_file,"."])
	
	#fname = sys.argv[1]
	fname_out = fname[0:len(fname)-4] + "_crop.pcd"

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

	lines = []

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
			x,y,z = (float(vals[0]),float(vals[1]),float(vals[2]))
			
			if x < minx: minx = x
			if x > maxx: maxx = x
			if y < miny: miny = y
			if y > maxy: maxy = y
			if z < minz: minz = z
			if z > maxz: maxz = z
			
			keep = True
			#keep = sqrt(x*x + y*y) < maxDistFromOrigin and z >= minHeight
			#keep = keep and not isnan(x)
			keep = keep and sqrt(x*x + y*y) < maxDistFromOrigin
			keep = keep and z <= -minHeight
			#keep = keep and z <= 0
			
			#l = vals[0] + " " + vals[1] + " " + str(-z) + " " + vals[3]
			
			if keep:
				lines.append(l + "\n")
				kept += 1
			else:
				reject += 1

	header = "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n" % (kept,kept)
	fout.write(header)

	for line in lines: fout.write(line)

	print "x: %f %f" % (minx,maxx)
	print "y: %f %f" % (miny,maxy)
	print "z: %f %f" % (minz,maxz)

	print "kept %d, rejected %d" % (kept,reject)
	
	f.close()
	fout.close()
	
	call(["rm",fname])
