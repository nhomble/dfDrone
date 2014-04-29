#!/usr/bin/env python2 

# python built-in modules
import math
import time

# SimpleCV imports
import SimpleCV

# my modules
import messageDrone

# eventually I should adapt these to parameters
MAX_DISTANCE = 200
MIN_RGB = 160
MAX_RGB = 210
MIN_AREA = 10000
MAX_AREA = 200000
DEBUG_STRING = "\t[DRONE_DETECT]"

MAX_BLOB_SIZE = .7
MIN_BLOB_SIZE = .1

EROSION = 30
BINARIZE = 90
MIN_CORNERS = 5
MAX_CORNERS = 41
MAX_LINES = 2
MIN_HOLES = 10

SQUARISH = .7

MATCH_STRENGTH = 5
MAX_SPLAY = 10
SPLAY_COUNT = 0
# make sure this is even
SPLAY_LENGTH = 2

LINE_FAILS = 1
CORNER_FAILS = 1
HOLE_FAILS = 4
COLOR_FAILS = 4
SQUARISH_FAILS = 5
MAX_FAILS = 4

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
			isFound, centroid, area, lines, corners, holes = self.hasDroneAux(img)
		else:	
			isFound, centroid, z, lines, corners, holes = self.hasDrone(img, depth)

		message = messageDrone.DFDMessage(isFound, centroid, z, area, img.width, img.height)
		#print(DEBUG_STRING + " " + str(message.isPresent))
		if isFound is True:
			print(DEBUG_STRING + " FOUND")
			print(DEBUG_STRING + " X:" + str(message.x) + " Y:" + str(message.y) + " Z:" + str(message.z))
			print(DEBUG_STRING + " Width:" + str(self.width) + " Height:" + str(self.height))
			print(DEBUG_STRING + " Area:" + str(area))
			print(DEBUG_STRING + " Lines: " + str(lines))
			print(DEBUG_STRING + " Corners: " + str(corners))
			print(DEBUG_STRING + " Holes: " + str(holes))
			#img.save(temp=True)
			return message
		return messageDrone.DFDMessage(False, None, None, None, img.width, img.height)
			

	# return whether we got something, and return the centroid if possible
	def hasDrone(self, img, depth):
		objects = None
		# when I have depth data, use it to:
		#	filter out object in the image
		#	get z
		flag, centroid, _, lines, corners, holes = self.hasDroneAux(img)
		if flag is True:
			z = depth.getPixel(int(math.floor(centroid[0])), int(math.floor(centroid[1])))
			if z[0] != 0:
				z = 1/z[0]
			else:
				z = -1
			return flag, centroid, z, lines, corners, holes
		else:
			return flag, None, None, None, None, None

	# helper function to see if an ARDRone is in an image by RGB
	def hasDroneAux(self, img):
		global SPLAY_COUNT

		if SPLAY_COUNT > MAX_SPLAY:
			self.foundBlobs = []
			SPLAY_COUNT = 0

		filtered = filterImage(img, self.debug)
		blobs = getBlobs(filtered, self.min_blob_size, self.max_blob_size)
		if blobs is None:
#			print(DEBUG_STRING + " no blobs to look at")
			return False, None, None, None, None, None

		for b in blobs:
			centroid = b.centroid()
			area = b.area()

			if area < MIN_AREA or area > MAX_AREA:
				print(DEBUG_STRING + " bad area " + str(area))
				break

			if self.blobAlreadySeen(b, img):
				SPLAY_COUNT += 1
				return True, centroid, area, -1, -1, -1

			cropped = cropFromBlob(b, img)
			
			# ignore bad blobs
			valid, lines, corners, holes = self.isValid(cropped)
			if valid is True:
				self.updateSeen((b, img))
				return True, centroid, area, lines, corners, holes
		self.foundBlobs = []
		return False, None, None, None, None, None

	# ok now I have a black blob, let's be clever
	def isValid(self, cropped):
		holes = []
		fails = 0

		retLines = 0
		retCorners = 0
		retHoles = 0

#		null check
		if cropped is None:
#			print(DEBUG_STRING + " nothing in crop")
			return False, None, None, None

#		I should not see that many, trying to avoid black squares/walls
		flag, lines = getLines(cropped)
		if flag is False:
			fails += LINE_FAILS
			print(DEBUG_STRING + " bad lines")
#			return False, None, None, None
		else:
			retLines = len(lines)

#		should see quite a few of these because of all the edges
		flag, corners = getCorners(cropped)
		if flag is False:
			fails += CORNER_FAILS
			print(DEBUG_STRING + " no corners")
#			return False, None, None, None
		else:
			retCorners = len(corners)

#		big distinguishing factor, there are holes for the rotors
#		as opposed to a TV
		flag, holes = getHoles(cropped)
		if flag is False:
			fails += HOLE_FAILS
			print(DEBUG_STRING + " no holes")
#			return False, None, None, None
		else:
			retHoles = len(holes)

#		gotta be black
		if validRGB(cropped.meanColor()):
			fails += COLOR_FAILS
			print(DEBUG_STRING + " average color of blob is not valid")
#			return False, None, None, None

#		try to avoid tall people
		if not self.squarish(cropped):
			fails += SQUARISH_FAILS
			print(DEBUG_STRING + " not squarish")
#			return False, None, None, None

		if fails > MAX_FAILS:
			print(DEBUG_STRING + " fails: " + str(fails))
			return False, None, None, None

		return True, retLines, retCorners, retHoles

	def squarish(self, cropped):
		div = float(cropped.width) / float(cropped.height)
		print(DEBUG_STRING + " " + str(div))
		if div < SQUARISH:
			return False
		else:
			return True

#	WAY TO SLOW!
	def percentChange(self, img1, img2):
		diff = img1 - img2
		matrix = diff.getNumpy()
		flat = matrix.flatten()
		counter = 0
		for i in flat:
			if flat[i] == 0:
				counter += 1

		return float(counter) / float(len(flat))


	def blobAlreadySeen(self, blob, img):
		counter = 0
		for tup in self.foundBlobs:
			if tup[0].match(blob) < MATCH_STRENGTH:
			#if self.percentChange(tup[1], img) < .10:
				# like splay
				self.foundBlobs.insert(0, self.foundBlobs.pop(counter))
				return True
			counter += 1
		self.foundBlobs = []
		return False

	def updateSeen(self, tup):
		self.foundBlobs.insert(0, tup)
		if len(self.foundBlobs) > SPLAY_LENGTH:
			self.foundBlobs = self.foundBlobs[:SPLAY_LENGTH/2]
	
# try to extract the darker parts of the image
def filterImage(img, debug):
	#img.show()
	#time.sleep(1)
	#gray = img.colorDistance(SimpleCV.Color.BLACK).dilate(3)
	#gray = gray/2
	#gray.show()
	#time.sleep(1)
	eroded = img.erode(EROSION)
	#eroded.show()
	#time.sleep(1)
	mult = eroded*2
	#mult.show()
	#time.sleep(1)
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
#		had unknown errors previously
#		simplecv bug?
		try:
			mean = l.meanColor()
		except:
			continue
		if validRGB(mean):
			ret.append(l)	
	if len(ret) < MAX_LINES:
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

# not finished
def getHoles(img):
	holes = img.findBlobs()
	if len(holes) > MIN_HOLES:
		count = 0
		for h in holes:
			if not validRGB(h.meanColor()) and str(h) != "":
				count += 1
		if count > MIN_HOLES:	
			return True, holes
	return False, None

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

