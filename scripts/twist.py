#!/usr/bin/env python2

# from ctypes import cdll

import sys
import threading
import time

import roslib; roslib.load_manifest('dfDrone')
import rospy
import tf.transformations
from geometry_msgs.msg import Twist

'''
need to review python threading!

with threads I hope to parallelize the queue of velocity vectors and
the processing of these vectors with twist messages to ROS 
'''
# cause data races are bad!
# use when accessing velocityQueue
queueLock = threading.Lock()
moveLock =threading.Lock()
class Control(object):
	def __init__(self):
		self.velocityQueue = []

		# start a thread to pop off velocity vectors 
		thread = popVelocityThread(self)	
		thread.start()
		thread.join()
	
	# spawn new threads
	def addToVelocityQueue(self, delX, delY, delZ):
		thread = addToQueueThread(self, delX, delY, delZ, delT)
		thread.start()
		thread.join()

	# TODO
	# we could make it smarted, but let's just spin right now
	def randomWalk(self):
		Twist.wheel_left.set_speed(1)
		Twist.wheel_right.set_speed(-1)


class popVelocityThread(threading.Thread):
	def __init__(self, control):
		threading.Thread.__init__(self, control)
		self.control = control

	def run(self):
		while True:
			popVelocityThreaded(self.control)

class addToQueueThread(threading.Thread):
	def __init__(self, control, delX, delY, delZ, delT):
		threading.Thread.__init__(self)
		self.control = control
		self.x = delX
		self.y = delY
		self.z = delZ
		self.t = delT
	
	def run(self):
		addToVelocityQueueThreaded(self.control, (self.x, self.y, self.z, self.t))

def addToVelocityQueueThreaded(control, velocity):
	with queueLock:
		control.velocityQueue.append(velocity)

def popVelocityThreaded(control):
	if len(control.velocityQueue) == 0:
		pass

	with queueLock:
		localData = threading.local()
		localData.velTuple = control.velocityQueue.pop()

	with moveLock:
		startTime = time.gmtime()
		# rotate
		radians = math.atan2(x, y)
		arcLength = math.pi * math.pow(localData.velTuple[2], 2)
		arcLength = arcLength * radians / (2 * math.pi)
		while localData.velTuple[3] > time.gmtime() - startTime:
			if localData.velTuple[2] < 0:
				Twist.wheel_right.set_speed(1/arcLength)
			elif localData.velTuple[2] > 0:
				Twist.wheel_left.set_speed(1/arcLength)
			else:
				break
			time.sleep(.5)

		startTime = time.gmtime()
		# move closer
		while localData.velTuple[3] > time.gmtime() - startTime:
			Twist.wheel_left.set_speed(1)
			Twist.wheel_right.set_speed(1)
			time.sleep(.5)