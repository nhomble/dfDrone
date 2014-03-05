#!/usr/bin/env python2

# TODO, translate into ROS

class DFDVelocity():
	def __init__(self, flag, linear, rotational, delay):
		self.isPresent = flag
		if self.isPresent is True:
			self.linear = linear
			self.rotational = rotational
			sel.delay = delay
