#!/usr/bin/env python2

import time

import SimpleCV

class Detector():
	def __init__(self, img):
		self.prevImg = img

		self.avgColor = img.getNumpy().mean()
		self.avgCount = 1

		self.lastSeenImg = None
		self.lastTime = None
	# give the detector another image that is 
	# hopefully representative of the environment
	# and then calculate the average color in the image
	def average(self, img):
		newVal = img.getNumpy().mean()
		self.avgColor = ((self.avgCount * self.avgColor) + newVal)/(self.avgCount + 1)
		self.avgCount += 1

	def process(self, img, depth):
		if(hasDrone(img, depth)):
			if(self.lastSeenImg is None):
				self.lastSeenImg = img
			if(self.lastTime is None):
				self.lastTime = time.gmtime()
		# eventually I should return a tuple of the new delX, delY	
			return True, None, None
		return False, None, None
			

	# the whole point of this class is to detect whether
	# a drone is in the image
	def hasDrone(self, img, depth):
		return false
	'''
	the following are wrappers of simplecv feature detection functions
	here I wanted to draw the features when found for debugging purposes
	and cool visuals
	'''
	def getLines(self, img):
		lines = img.findLines()
		if(lines is not None):
			lines.draw()
		return lines

	def getCorners(self, img):
		corners = img.findCorners()
		if(corners is not None):
			corners.draw()
		return corners

	def getBlobs(self, img):
		blobs = img.findBlobs()
		if(blobs is not None):
			blobs.draw()
		return blobs
