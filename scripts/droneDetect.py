#!/usr/bin/env python2 

import math
import time

import SimpleCV
import scipy

# called by dfDrone to request information
class Detector():
	def __init__(self, img, useKinect, debug):
		self.debug = debug
		self.useKinect = useKinect
	
		self.width = img.height
		self.height = img.width
		self.screensize = self.height * self.width

		# precision in blob detection
		self.min_blob_size = .05 * self.screensize
		self.max_blob_size = .7 * self.screensize

		# I want to keep the centroid in the center of the image
		self.lastSeenCent = (img.width/2, img.height/2)

		# TODO may need to change
		self.lastDepth = 0
		self.lastTime = None

		# use previously found blobs and compare!!!
		self.foundBlobs = []

	# called by dfDrone
	# we determine if we see a drone in the image
	# if we see the image, then we determine the delta(x, y, z)
	def process(self, img, depth):
		isFound, centroid, dep = self.hasDrone(img, depth)
		if isFound is True:
			if(self.lastTime is None):
				self.lastTime = time.time()

			# create a velocity vector
			delTime = time.time() - self.lastTime
			delX = centroid[0] - self.lastSeenCent[0]
			delX = delX/delTime
			delY = centroid[1] - self.lastSeenCent[1]
			delY = delY//delTime

			self.lastSeenCent = centroid

			if dep is not None:
				delZ = (dep - self.lastDepth)/delTime	
			else:
				delZ = 0
			return True, math.ceil(delX), math.ceil(delY), math.ceil(delZ), math.ceil(delTime)
		return False, None, None, None, None
			

	# return whether we got something, and return the centroid if possible
	def hasDrone(self, img, depth):
		objects = None

		# when I have depth data, use it to:
		#	filter out object in the image
		#	get z
		if depth is not None:
			# no objects means no ARDRone
			objects = getBlobs(depth, self.min_blob_size, self.max_blob_size)
			if objects is None:
				return False, None, None
			
			# each image, crop out the blob and then analyze by RGB
			for obj in objects:
				cropped = cropFromBlobs(obj, img)
				
				# see if the object is an ARDrone
				tValid, tCentroid= self.hasDroneAux(cropped)
				if tValid is True:
					# index the depth matrix with our centroid
					dep = depth.getPixel(tCentroid[0], tCentroid[1])
					# calibrate dep TODO
					return tValid, tCentroid, dep
			# bail, try the next image
			return False, None, None
		# debug, dno depth data
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
			# have I seen a blob like this before
			if self.blobAlreadySeen(b):
				return True, centroid
			
			cropped = cropFromBlob(b, img)

			# get rid of bad crop
			if cropped is None:
				continue

			if self.isValid(cropped, centroid):
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

	def blobAlreadySeen(self, blob):
		counter = 0
		for b in self.foundBlobs:
			# not sure how precise TODO
			#if b.match(blob) < 10:
				# like splay
			#	return True
			counter += 1
		return False

def filterImage(img, debug):
	# try to extract the darker parts of the image
	mult = img*4
	eroded = mult.erode(10)
	div = eroded/2
	binary = div.binarize(90)
	if debug is True:
		img.show()
		time.sleep(1)
		mult.show()
		time.sleep(1)
		eroded.show()
		time.sleep(1)
		div.show()
		time.sleep(1)
		binary.show()
		time.sleep(1)
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
		# if we get just ONE! black line let's go with it
		if validRGB(mean):
			ret.append(l)	
	if len(ret) > 0:
		return True, ret
	return False, None

# should be black as well
# return a list of valid corners
def validCorners(corners):
	if corners is None:
		return False, None

	if len(corners) > 0:
		numValid = 0
		for c in corners:
			if validRGB(c.meanColor()):
				numValid += 1
		if numValid >= len(corners)/5:
			return True, corners
		return False, None
		 
	return False, None
'''
the following are wrappers of simplecv feature detection functions
here I wanted to draw the features when found for debugging purposes
and cool visuals
'''
def getLines(img):
	lines = img.findLines()
#	if(lines is not None):
#		lines.draw()
	return validLines(lines)

def getCorners(img):
	corners = img.findCorners()
#	if(corners is not None):
#		corners.draw()
	return validCorners(corners)

def getBlobs(img, bMin, bMax):
	blobs = None
	try:
		blobs = img.findBlobs(minsize=bMin, maxsize=bMax)
	except:
		print("find blobs complained")
#	if(blobs is not None):
#		blobs.draw()
	return blobs

