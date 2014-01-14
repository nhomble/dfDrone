#!/usr/bin/env python2

import math
import time

import SimpleCV
import scipy

# called by dfDrone to request information
class Detector():
	def __init__(self, img, useKinect):
		self.useKinect = useKinect

		self.width = img.height
		self.height = img.width

		self.lastSeenCent = None
		self.lastDepth = None
		self.lastTime = None

		# use previously found blobs and compare!!!
		self.foundBlobs = []

	# called by dfDrone
	# we determine if we see a drone in the image
	# if we see the image, then we determine the delta(x, y)
	def process(self, img, depth):
		isFound, centroid, dep = hasDrone(img, depth)
		if isFound is True:
			if(self.lastSeenCent is None):
				self.lastSeenCent = centroid
			if(self.lastDepth is None):
				self.lastDepth = dep
			if(self.lastTime is None):
				self.lastTime = time.gmtime()
			# create a velocity vector
			delTime = time.gmtime() - self.lastTime
			delX = (centroid[0] - self.lastSeenCent[0])/delTime
			delY = (centroid[1] - self.lastSeenCent[1])/delTime
			delZ = (dep - self.lastDepth)/delTime	
			return True, math.ceil(delX), math.ceil(delY), math.ceil(delZ)
		return False, None, None, None
			

	# return whether we got something, and return the centroid if possible
	def hasDrone(self, img, depth):
		# try to extract the darker parts of the image
		binary = img.binarize(85)
		blobs = self.getBlobs(binary)
		if blobs is None:
			return False, None
		for b in blobs:
			if self.blobAlreadySeen(b):
				return True, b.centroid()

			# not clear which to use - not very intuitive
			x = math.floor(b.minRectX())
			dx = math.ceil(b.minRectWidth())
			y = math.floor(b.minRectY())
			dy = math.floor(b.minRectHeight())
			cx = b.centroid()[0]
			cy = b.centroid()[1]
			cropped = img.crop(cx-dx, cy-dy, 2*dx, 2*dy)
			if b.area() > 200 and self.isValid(cropped):
				self.foundBlobs.append(b)
				return True, b.centroid()
		return False, None

	# ok now I have a black blob, let's be clever and look at its edges
	def isValid(self, cropped):
		cropped.show()
		time.sleep(2)
		return False

	def blobAlreadySeen(self, blob):
		counter = 0
		for b in self.foundBlobs:
			# not sure how precise TODO
			if b.match(blob) < 10:
				# like splay
				self.foundBlobs.pop(counter)
				self.foundBlobs.insert(b)	
				return True
			counter += 1
		return False

	'''
	when I detect features I want them to meet certain criteria
	'''
	# represents structure/rotors the color should be black
	# return a list of valid lines
	def validLines(self, lines):
		ret = []
		if lines is None:
			return False, None
		for l in lines:
			mean = l.meanColor()
			# if we get just ONE! black line let's go with it
			if validRGB(mean):
				ret.append(l)	
		if len(ret) > 0:
			return True, ret
		return False, None

	# should be black as well
	# return a lis of valid corners
	def validCorners(self, corners):
		if corners is None:
			return False, None
		if len(corners) > 0:
			return True, corners 
		return False, None
	'''
	the following are wrappers of simplecv feature detection functions
	here I wanted to draw the features when found for debugging purposes
	and cool visuals
	'''
	def getLines(self, img):
		lines = img.findLines()
		if(lines is not None):
			lines.draw()
		return self.validLines(lines)

	def getCorners(self, img):
		corners = img.findCorners()
		if(corners is not None):
			corners.draw()
		return self.validCorners(corners)

	def getBlobs(self, img):
		blobs = img.findBlobs()
		if(blobs is not None):
			blobs.draw()
		return blobs

# I want to detect black which inverted is white
def validRGB(rgb):
	if rgb[0] >= 225 and rgb[1] >= 225 and rgb[2] >= 225:
		return True
	return False
