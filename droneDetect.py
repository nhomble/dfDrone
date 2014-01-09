#!/usr/bin/env python2

import time

import SimpleCV

# called by dfDrone to request information
class Detector():
	def __init__(self, img, useKinect):
		self.useKinect = useKinect

		self.prevImg = img

		self.width = img.height
		self.height = img.width

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
			

	'''
	use blob areas to only consider blobs of certain size - in conjunction with
	depth sensor we know how the image should dilate...

	we can crop out the minimal rectangle	
	'''
	def hasDrone(self, img, depth):
		blobs = self.getBlobs(img)
		if blobs is None:
			return False
		for b in blobs:
			'''
			print(b)
			print("minrect ", b.minRect())
			print("min", b.minRectHeight(), b.minRectWidth())
			print("minCoodinates ", b.minRectX(), b.minRectY())
			print("area ", b.area())
			print("circle ", b.isCircle())
			print("rectangle ", b.isRectangle())
			print("square ", b.isSquare())
			print("mean color ", b.meanColor())
			print("perimeter ", b.perimeter())
			print("\n")
			'''
			cropped = img.crop(b.minRectX(), b.minRectY(), b.minRectX() + b.minRectWidth(), b.minRectY() + b.minRectHeight())
			validLines, lines = self.getLines(cropped)
			corners = self.getCorners(cropped)
			time.sleep(2)
		return False

	'''
	when I detect features I want them to meet certain criteria
	'''
	# represents structure/rotors the color should be black
	def validLines(self, lines):
		if lines is None:
			return False, None
		for l in lines:
			mean = l.meanColor()
			# if we get just ONE! black line let's go with it
			if mean[0] <= 75 and mean[1] <= 75 and mean[2] <= 75:
				return True, lines
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
		return corners

	def getBlobs(self, img):
		blobs = img.findBlobs()
		if(blobs is not None):
			blobs.draw()
		return blobs
