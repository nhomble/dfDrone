#!/usr/bin/env python2

# TODO, translate into ROS

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
