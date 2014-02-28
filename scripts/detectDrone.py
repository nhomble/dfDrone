#!/usr/bin/env python2 

import math
import time

import SimpleCV
import scipy

# called by dfDrone to request information
class Detector():
	def __init__(self, debug):
		self.debug = debug

		self.min_blob_size = None
		self.max_blob_size = None
	
		# use previously found blobs and compare!!!
		self.foundBlobs = []

	# we determine if we see a drone in the image
	# if we see the image, then we determine the delta(x, y, z)
	def process(self, img, depth):
		if (img is None or depth is None) and self.debug is False:
			return False, None, None

		self.min_blob_size = .05 * img.height
		self.max_blob_size = .7 * img.height
		
		isFound, centroid, z = self.hasDrone(img, depth)
		if isFound is True:
			if self.debug is True:
				print("FOUND")
			
			return True, centroid, z
		return False, None, None
			

	# return whether we got something, and return the centroid if possible
	def hasDrone(self, img, depth):
		objects = None

		# when I have depth data, use it to:
		#	filter out object in the image
		#	get z
		if depth is not None:
			objects = getBlobs(depth, self.min_blob_size, self.max_blob_size)
			if objects is None:
				return False, None, None
			
			for obj in objects:
				cropped = cropFromBlobs(obj, img)
				
				tValid, tCentroid= self.hasDroneAux(cropped)
				if tValid is True:
					dep = depth.getPixel(tCentroid[0], tCentroid[1])
					# calibrate dep TODO
					return tValid, tCentroid, dep
			return False, None, None
		else:
			v, c = self.hasDroneAux(img)
			return v, c, None

	# helper function to see if an ARDRone is in an image by RGB
	def hasDroneAux(self, img):
		filtered = filterImage(img, self.debug)
		blobs = getBlobs(filtered, self.min_blob_size, self.max_blob_size)
		if blobs is None:
			return False, None

		for b in blobs:
			centroid = b.centroid()
			if self.blobAlreadySeen(b):
				return True, centroid
			
			cropped = cropFromBlob(b, img)

			if cropped is None:
				continue

			if self.isValid(cropped, centroid):

				if cropped.area() > 20000 and len(blobs) > 1:
					return self.hasDroneAux(cropped)
				# TODO hardcode
				self.foundBlobs.append(b)
				return True, centroid
		return False, None

	# ok now I have a black blob, let's be clever
	# check area - check hue peaks
	# TODO
	def isValid(self, cropped, centroid):
		if cropped is None:
			return False
		# because of the frame of the drone I should see SOME corners
		flag, corners = getCorners(cropped)
		if flag is False:
			return False

		if self.debug is True:
			for c in corners:
				c.draw()
			cropped.show()
			time.sleep(2)
		return True

	# TODO fix
	def blobAlreadySeen(self, blob):
		return False
		counter = 0
		for b in self.foundBlobs:
			# not sure how precise TODO
			if b.match(blob) < 10:
				# like splay
				return True
			counter += 1
		return False


# try to extract the darker parts of the image
def filterImage(img, debug):
	gray = img.colorDistance(SimpleCV.Color.BLACK).dilate(3)
	gray = gray/2
	eroded = gray.erode(10)
	mult = eroded*2
	binary = mult.binarize(90)
	if debug is True:
		img.show()
		time.sleep(.5)
		gray.show()
		time.sleep(.5)
		eroded.show()
		time.sleep(.5)

		mult.show()
		time.sleep(.5)
		#div.show()
		#time.sleep(.5)
		binary.show()
		time.sleep(.5)
	return binary

# just return a cropped image from a blob
def cropFromBlob(blob, image):
	cent = blob.centroid()
	dx = blob.minRectWidth()/2
	dy = blob.minRectHeight()/2
	mX = blob.minRectX()-dx
	mY = blob.minRectY()-dy

	if dx < 50 and dy < 50:
		return None

	if mX < 0:
		mX = 0
	if mY < 0:
		mY = 0

	cropped = image.crop(mX, mY, 2*dx, 2*dy)
	
	return cropped


# I want to detect black which inverted is white
def validRGB(rgb):
	if rgb[0] >= 100 and rgb[1] >= 100 and rgb[2] >= 100:
		return True
	return False

'''
when I detect features I want them to meet certain criteria
'''
# represents structure/rotors the color should be black
# return a list of valid lines
def validLines(lines):
	ret = []
	if lines is None:
		return False, None
	for l in lines:
		mean = l.meanColor()
		if validRGB(mean):
			ret.append(l)	
	if len(ret) > 0:
		return True, ret
	return False, None

# should be black as well
# return a list of valid corners
def validCorners(corners, img):
	if corners is None:
		return False, None

	if len(corners) > 0:
		numValid = 0
		for c in corners:
			dist = distanceFromCenter(c, img)
			# print(dist)
			# TODO I should not hardcode this value
			if dist > 75:
				continue
			
			if validRGB(c.meanColor()):
				numValid += 1
		# TODO should not hardcode
		if numValid > 5 and numValid < 15:
			return True, corners
		return False, None
		 
	return False, None

# obj must have x, y fields
def distanceFromCenter(obj, img):
	width = img.width
	height = img.height
	center = (width/2, height/2 )

	delX = obj.x - center[0]
	delY = obj.y - center[1]

	distance = (delX*delX) + (delY*delY)
	distance = math.sqrt(distance)

	return distance

'''
the following are wrappers of simplecv feature detection functions
here I wanted to draw the features when found for debugging purposes
and cool visuals
'''
def getLines(img):
	lines = img.findLines()
	if(lines is not None):
		lines.draw()
	return validLines(lines)

def getCorners(img):
	corners = img.findCorners()
	if(corners is not None):
		corners.draw()
	return validCorners(corners, img)

def getBlobs(img, bMin, bMax):
	blobs = None
	try:
		blobs = img.findBlobs(minsize=bMin, maxsize=bMax)
	except:
		print("find blobs complained")
#	if(blobs is not None):
#		blobs.draw()
	return blobs

