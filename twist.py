#!/usr/bin/env python2

from ctypes import cdll
import sys
import threading

'''
need to review python threading!

with threads I hope to parallelize the queue of velocity vectors and
the processing of these vectors with twist messages to ROS (via cytpes)
'''
# cause data races are bad!
# use when accessing velocityQueue
lock = Thread.Lock()
class Control(object):
	def __init__(self):
			
		try:
			open('./libtwist', 'r')
			pass
		except IOError:
			print("where is libtwist?, make sure the file is compiled and in the same directory")
		self.lib = cdll.LoadLibrary('./libtwist')
		self.twist = self.lib.Twist_new()

		self.velocityQueue = []

		# start thread to spawn more threads to pop velocity vectors
		thread = threading.Thread(target=popVelocity, args=(self, ))	
		thread.start()
		thread.join()
	
	# spawn thread to add velocity vector to queue
	def addToVelocityQueue(self, delX, delY):
		thread = threading.Thread(target=addToVelocityQueueThreaded, args=(self, (delX, delY)))
		thread.start()
		thread.join()

def addToVelocityQueueThreaded(control, velocity):
	with lock:
		control.velocityQueue.append(velocity)

def popVelocity(control):
	while True:
		thread = threading.Thread(target=popVelocityThreaded, args=(control, ))
		thread.start()
		thread.join()

def popVelocityThreaded(control):
	if len(control.velocityQueue) == 0:
		pass

	with lock:
		# send twist message via ctypes
		velTuple = control.velocityQueue.pop()

	control.lib.twist_sendMessage(control.twist, velTuple[0], velTuple[1], velTuple[2]);
