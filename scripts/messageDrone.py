#!/usr/bin/env python2

try:
	from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
except:
	print("ROS installed? I could not import std_msgs in messageDrone")

class DFDVelocity():
	def __init__(self, flag, delx, dely, z):
		self.isPresent = flag
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0

		if self.isPresent is True:
			self.x = float(delx)
			self.y = float(dely)
			self.z = float(z)
	
#	I need to look around quickly, but I do not necessarily need to move forward
	def amplify(self, amp):
		self.x *= amp
		self.y *= amp
		#self.z *= amp

	def ros(self):
		message = Float64MultiArray()
		flag = 0.0
		if self.isPresent is True:
			flag = 1.0
		message.data = [flag, self.x, self.y, self.z]
		return message
	
class DFDMessage():
	def __init__(self, flag, centroid, z, area, w, h):
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
		self.area = 0.0
		if self.isPresent is True:
			self.x = float(centroid[0])
			self.y = float(centroid[1])
			if z is not None:
				self.z = float(z)
			if area is not None:
				self.area = float(area)
	def __str__(self):
		return "" + self.isPresent
	
	def ros(self):
		message = Float64MultiArray()

		boolean = 0.0
		if self.isPresent is True:
			boolean = 1.0

		message.data = [boolean, self.x, self.y, self.z, self.area, self.width, self.height]

		return message
