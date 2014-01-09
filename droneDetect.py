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

		self.lastSeenCent = None
		self.lastTime = None

	# called by dfDrone
	# we determine if we see a drone in the image
	# if we see the image, then we determine the delta(x, y)
	def process(self, img, depth):
		isFound, centroid = hasDrone(img, depth)
		if isFound is True:
			if(self.lastSeenCent is None):
				self.lastSeenImg = centroid
			if(self.lastTime is None):
				self.lastTime = time.gmtime()
		# eventually I should return a tuple of the new delX, delY	
			return True, None, None
		return False, None, None
			

	# return whether we got something, and return the centroid if possible
	def hasDrone(self, img, depth):
		# highlight the black image to white!
		img = img.invert()
		blobs = self.getBlobs(img)
		if blobs is None:
			return False, None
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
			validCorners, corners = self.getCorners(cropped)
			# then if I have enough features that meet my criteria I can do
			# further analysis and then determine if we see a similar looking
			# object is the actual object
			if validLines or validCorners:
				b.show()
				time.sleep(5)
				return True, b.centroid()
		return False, None

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
		ret = []
		if corners is None:
			return False, None
		for c in corners:
			mean = c.meanColor()
			if validRGB(mean):
				ret.append(c)
		if len(ret) > 0:
			return True, ret
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
