#!/usr/bin/env python2 

# python built-in modules
import math
import time

# SimpleCV imports
import SimpleCV

# my modules
import messageDrone

# "ENUMS"
MIN_DISTANCE = 150
MIN_RGB = 100
MAX_RGB = 140
MIN_AREA = 20000
DEBUG_STRING = "\t[DRONE_DETECT]"
DEBUG = False

# called by dfDrone to request information
class Detector():
	def __init__(self, debug):
		self.debug = debug
		DEBUG = self.debug

		self.min_blob_size = None
		self.max_blob_size = None
	
		self.width = None
		self.height = None

		# use previously found blobs and compare!!!
		self.foundBlobs = []

	# we determine if we see a drone in the image
	# if we see the image, then we determine the delta(x, y, z)
	def process(self, img, depth):
		if (img is None) and self.debug is False:
			return messageDrone.DFDMessage(False, None, None, None)

		self.width = img.width
		self.height = img.height

		self.min_blob_size = .05 * img.height * img.width
		self.max_blob_size = .7 * img.height * img.width
	
		if depth is None:
			isFound, centroid = self.hasDroneAux(img, 0)
			z = 100
		else:	
			isFound, centroid, z = self.hasDrone(img, depth)

		message = messageDrone.DFDMessage(isFound, centroid, z, img.width, img.height)
		print(DEBUG_STRING + " " + str(message.isPresent))
		if isFound is True:
			if self.debug is True:
				print(DEBUG_STRING + " FOUND")
				print(DEBUG_STRING + " " + str(message.x) + " " + str(message.y))
				print(DEBUG_STRING + " " + str(self.width) + " " + str(self.height))
			
			return message
		return messageDrone.DFDMessage(False, None, None, None, None)
			

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
				return tValid, tCentroid, dep
				if tValid is True:
					dep = depth.getPixel(tCentroid[0], tCentroid[1])
					# calibrate dep TODO
					return tValid, tCentroid, dep
			return False, None, None
		else:
			v, c = self.hasDroneAux(img, 0)
			return v, c, None

	# helper function to see if an ARDRone is in an image by RGB
	def hasDroneAux(self, img, iterations):
		filtered = filterImage(img, self.debug)
		blobs = getBlobs(filtered, self.min_blob_size, self.max_blob_size)
		if blobs is None:
			print(DEBUG_STRING + " no blobs to look at")
			return False, None

		for b in blobs:
			centroid = b.centroid()
			if self.blobAlreadySeen(b):
				return True, centroid
			
			cropped = cropFromBlob(b, img)

			if cropped is None:
				continue

			# ignore bad bloba
			if self.isValid(cropped, centroid):
				# if cropped.area() > MIN_AREA and len(blobs) > 1:
				self.foundBlobs.append(b)
				return True, centroid
				# hmm it is suspiciously large
				# else:	
				#	if iterations < 3:
				#		if self.debug is True:
				#			print(DEBUG_STRING + " recurse detection")
				#		tFlag, tCent = self.hasDroneAux(cropped, iterations + 1)
				#		if tFlag is False:
				#			return True, centroid
		return False, None

	# ok now I have a black blob, let's be clever
	# check area - check hue peaks
	# TODO
	def isValid(self, cropped, centroid):
		if cropped is None:
			print(DEBUG_STRING + " nothing in crop")
			return False
		# because of the frame of the drone I should see SOME corners
		flag, corners = getCorners(cropped)
		if flag is False:
			print(DEBUG_STRING + " no corners")
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
	if rgb[0] >= MIN_RGB and rgb[1] >= MIN_RGB and rgb[2] >= MIN_RGB:
		if rgb[0] <= MAX_RGB and rgb[1] <= MAX_RGB and rgb[2] <= MAX_RGB:
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
			if dist > MIN_DISTANCE:
				if DEBUG is True:
					print("" + DEBUG_STRING + " corner is too far from center " + str(dist))
				continue
			
			if validRGB(c.meanColor()):
				if DEBUG is True:
					print(DEBUG_STRING + " valid rgb + " + str(c.meanColor()))
				numValid += 1
			else:
				if DEBUG is True:
					print("" + DEBUG_STRING + " corner is not a valid color " + str(c.meanColor()))
		# TODO should not hardcode
		if numValid > 5 and numValid < 20:
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
		print("" + DEBUG_STRING + " find blobs complained")
#	if(blobs is not None):
#		blobs.draw()
	return blobs

