#!/usr/bin/env python2

from std_msgs.msg import Float64MultiArray

class DFDVelocity():
	def __init__(self, flag, delx, dely, z, delay):
		self.isPresent = flag
		self.delX = None
		self.delY = None
		self.z = None
		self.delay = None
		if self.isPresent is True:
			self.delX = delx
			self.delY = dely
			self.z = z
			sel.delay = delay

	def ros(self):
		message = Float64MultiArray()
		message.layout.dim[0].label = "velocity"
		message.layout.dim[0].size = 5
		message.layout.dim[0].stride = 1

		message.layout.dim[1].label = "junk"
		message.layout.dim[1].size = 0
		message.layout.dim[1].stride = 0

		message.layout.dim[2].label = "junk"
		message.layout.dim[2].size = 0
		message.layout.dim[3].size = 0

		boolean = 0.0
		if self.isPresent is True:
			boolean = 1.0

		message.data = [boolean, self.delX, self.delY, self.z, self.delay]

		return message
