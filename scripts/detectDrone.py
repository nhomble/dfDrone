#!/usr/bin/env python2 

# python built-in modules
import math
import time

# SimpleCV imports
import SimpleCV

# my modules
import messageDrone

# "ENUMS"
MAX_DISTANCE = 200
MIN_RGB = 100
MAX_RGB = 210
MIN_AREA = 1000
MAX_AREA = 75000
DEBUG_STRING = "\t[DRONE_DETECT]"

MAX_BLOB_SIZE = .7
MIN_BLOB_SIZE = .05

EROSION = 10
BINARIZE = 90
MIN_CORNERS = 10
MAX_CORNERS = 20

SQUARISH = .7

# different constants for inside/outside net
if False:
	pass

# depracated flag
DEBUG = True

# called by dfDrone to request information
class Detector():
	def __init__(self):
		self.debug = DEBUG
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

		self.min_blob_size = MIN_BLOB_SIZE * img.height * img.width
		self.max_blob_size = MAX_BLOB_SIZE * img.height * img.width
	
		area = 0.0
		z = 0.0
		if depth is None:
			isFound, centroid, area = self.hasDroneAux(img)
		else:	
			isFound, centroid, z = self.hasDrone(img, depth)

		message = messageDrone.DFDMessage(isFound, centroid, z, area, img.width, img.height)
		#print(DEBUG_STRING + " " + str(message.isPresent))
		if isFound is True:
			print(DEBUG_STRING + " FOUND")
			print(DEBUG_STRING + " X:" + str(message.x) + " Y:" + str(message.y))
			print(DEBUG_STRING + " Width:" + str(self.width) + " Height:" + str(self.height))
			print(DEBUG_STRING + " Area:" + str(area))
			#img.save(temp=True)
			return message
		return messageDrone.DFDMessage(False, None, None, None, img.width, img.height)
			

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
			v, c, area = self.hasDroneAux(img, 0)
			return v, c, area

	# helper function to see if an ARDRone is in an image by RGB
	def hasDroneAux(self, img):
		filtered = filterImage(img, self.debug)
		blobs = getBlobs(filtered, self.min_blob_size, self.max_blob_size)
		if blobs is None:
			#print(DEBUG_STRING + " no blobs to look at")
			return False, None, None

		for b in blobs:
			centroid = b.centroid()
			cropped = cropFromBlob(b, img)
			area = b.area()
			if self.blobAlreadySeen(b, img):
				return True, centroid, area
			
			# ignore bad blobs
			if self.isValid(cropped, centroid):
				# if cropped.area() > MIN_AREA and len(blobs) > 1:
				self.updateSeen((b, img))
				return True, centroid, area
				# hmm it is suspiciously large
				# else:	
				#	if iterations < 3:
				#		if self.debug is True:
				#			print(DEBUG_STRING + " recurse detection")
				#		tFlag, tCent = self.hasDroneAux(cropped, iterations + 1)
				#		if tFlag is False:
				#			return True, centroid
		return False, None, None

	# ok now I have a black blob, let's be clever
	# check area - check hue peaks
	def isValid(self, cropped, centroid):
		if cropped is None:
			print(DEBUG_STRING + " nothing in crop")
			return False
		flag, corners = getCorners(cropped)
		if flag is False:
			print(DEBUG_STRING + " no corners")
			return False

		if validRGB(cropped.meanColor()):
			print(DEBUG_STRING + " average color of blob is not valid")
			return False

		if cropped.area() > MAX_AREA or cropped.area() < MIN_AREA:
			print(DEBUG_STRING + " bad area " + str(cropped.area()))
			return False

		if not self.squarish(cropped):
			print(DEBUG_STRING + " not squarish")
			return False
		return True

	def squarish(self, cropped):
		div = float(cropped.width) / float(cropped.height)
		print(DEBUG_STRING + " " + str(div))
		if div < SQUARISH:
			return False
		else:
			return True

	def blobAlreadySeen(self, blob, img):
		counter = 0
		for tup in self.foundBlobs:
			if tup[0].match(blob) < 10:
				# like splay
				self.foundBlobs.insert(0, self.foundBlobs.pop(counter))
				
				return True
			counter += 1
		return False

	def updateSeen(self, tup):
		self.foundBlobs.insert(0, tup)
		if len(self.foundBlobs) > 10:
			self.foundBlobs = self.foundBlobs[:5]
	

	def percentChange(self, img1, img2):
		diff = img1 - img2
		matrix = diff.getNumpy()
		flat = matrix.flatten()
		counter = 0
		for i in flat:
			if flat[i] == 0:
				counter += 1

		return float(counter) / float(len(flat))

# try to extract the darker parts of the image
def filterImage(img, debug):
	gray = img.colorDistance(SimpleCV.Color.BLACK).dilate(3)
	gray = gray/2
	eroded = gray.erode(EROSION)
	mult = eroded*2
	binary = mult.binarize(BINARIZE)
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
			if dist > MAX_DISTANCE:
				continue
			
			if validRGB(c.meanColor()):
				numValid += 1
		# TODO should not hardcode
		if numValid > MIN_CORNERS and numValid < MAX_CORNERS:
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

