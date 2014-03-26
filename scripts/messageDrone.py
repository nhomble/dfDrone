#!/usr/bin/env python2

try:
	from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
except:
	print("ROS installed? I could not import std_msgs in messageDrone")

class DFDMessage():
	def __init__(self, flag, centroid, z, w, h):
		self.isPresent = flag
		self.width  = w
		self.height = h
		self.x = 0
		self.y = 0
		self.z = 0
		if self.isPresent is True:
			self.x = centroid[0]
			self.y = centroid[1]
			self.z = z
	def __str__(self):
		return "" + self.isPresent
	
	def ros(self):
		message = Float64MultiArray()
		message.layout = MultiArrayLayout()
		message.layout.dim = MultiArrayDimension()

		message.layout.dim.label = "centroid"
		message.layout.dim.size = 5
		message.layout.dim.stride = 1

		boolean = 0.0
		if self.isPresent is True:
			boolean = 1.0

		message.data = [boolean, self.x, self.y, self.z, self.width, self.height]

		return message
