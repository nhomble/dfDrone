#!/usr/bin/env python2

from std_msgs.msg import Float64MultiArray

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
			sel.delay = float(delay)

	def ros(self):
		message = Float64MultiArray()
		
		boolean = 0.0
		if self.isPresent is True:
			boolean = 1.0

		message.data = [boolean, self.delX, self.delY, self.z, self.delay]

		return message
