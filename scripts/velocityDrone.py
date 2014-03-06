#!/usr/bin/env python2

# TODO, translate into ROS

class DFDVelocity():
	def __init__(self, flag, delx, dely, z, delay):
		self.isPresent = flag
		self.delX = None
		self.delY = None
		self.z = None
		sel.delay = None
		if self.isPresent is True:
			self.delX = delx
			self.delY = dely
			self.z = z
			sel.delay = delay
