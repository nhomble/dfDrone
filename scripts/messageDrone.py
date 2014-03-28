#!/usr/bin/env python2

try:
	from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
except:
	print("ROS installed? I could not import std_msgs in messageDrone")

class DFDVelocity():
	def __init__(self, flag, delx, dely, z, delay):
		self.isPresent = flag
		self.delX = 0.0
		self.delY = 0.0
		self.z = 0.0
		self.delay = 0.0
		if self.isPresent is True:
			self.delX = float(delx)
			self.delY = float(dely)
			self.z = float(z)
			self.delay = float(delay)

	def ros(self):
		message = Float64MultiArray()
		
		boolean = 0.0
		if self.isPresent is True:
			boolean = 1.0

		message.data = [boolean, self.delX, self.delY, self.z, self.delay]

		return message
	
class DFDMessage():
	def __init__(self, flag, centroid, z, w, h):
		self.isPresent = flag
	
		if (w and h) is not None:
			self.width  = float(w)
			self.height = float(h)
		else:
			self.width = 0.0
			self.height = 0.0
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		if self.isPresent is True:
			self.x = centroid[0]
			self.y = centroid[1]
			self.z = z
	def __str__(self):
		return "" + self.isPresent
	
	def ros(self):
		message = Float64MultiArray()

		boolean = 0.0
		if self.isPresent is True:
			boolean = 1.0

		message.data = [boolean, self.x, self.y, self.z, self.width, self.height]

		return message
