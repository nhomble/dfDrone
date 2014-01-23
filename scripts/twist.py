#!/usr/bin/env python2

from ctypes import cdll
import sys
import threading

'''
need to review python threading!
	`	`
with threads I hope to parallelize the queue of velocity vectors and
the processing of these vectors with twist messages to ROS 
'''
# cause data races are bad!
# use when accessing velocityQueue
lock = threading.Lock()
class Control(object):
	def __init__(self):
		self.velocityQueue = []

		# start thread to spawn more threads to pop velocity vectors
		thread = popVelocityThread(self)	
		thread.start()
		thread.join()
	
	# spawn thread to add velocity vector to queue
	def addToVelocityQueue(self, delX, delY, delZ):
		thread = addToQueueThread(self, delX, delY, delZ)
		thread.start()
		thread.join()


class popVelocityThread(threading.Thread):
	def __init__(self, control):
		threading.Thread.__init__(self)
		self.control = control

	def run(self):
		while True:
			popVelocityThreaded(self.control)

class addToQueueThread(threading.Thread):
	def __init__(self, control, delX, delY, delZ):
		threading.Thread.__init__(self)
		self.control = control
		self.x = delX
		self.y = delY
		self.z = delZ
	def run(self):
		addToVelocityQueueThreaded(self.control, (self.x, self.y, self.z))

def addToVelocityQueueThreaded(control, velocity):
	with lock:
		control.velocityQueue.append(velocity)

def popVelocityThreaded(control):
	if len(control.velocityQueue) == 0:
		pass

	with lock:
		# send twist message via ctypes
		velTuple = control.velocityQueue.pop()

	control.lib.twist_sendMessage(control.twist, velTuple[0], velTuple[1], velTuple[2]);
