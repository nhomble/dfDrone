#!/usr/bin/env python2

class DFDMessage():
	def __init__(self, flag, centroid, z):
		self.isPresent = flag
		if self.isPresent is True:
			self.x = centroid[0]
			self.y = centroid[1]
			self.z = z
	def __str__(self):
		return "" + self.isPresent
