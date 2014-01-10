#!/usr/bin/env python2

from ctypes import cdll
import sys
import threading

class Control(object):
	def __init__(self):
			
		try:
			open('./libtwist', 'r')
			pass
		except IOError:
			print("where is libtwist?, make sure the file is compiled and in the same directory")
		self.lib = cdll.LoadLibrary('./libtwist')
		self.velocityQueue = []
		thread = threading.Thread(target=popVelocity, args=(self, ))	
		thread.start()
		thread.join()
	
	def addToVelocityQueue(self, delX, delY):
		thread = threading.Thread(target=addToVelocityQueueThreaded, args=(self, (delX, delY)))
		thread.start()
		thread.join()

def addToVelocityQueueThreaded(control, velocity):
	control.velocityQueue.append(velocity)

def popVelocity(control):
	while True:
		thread = threading.Thread(target=popVelocityThreaded, args=(control, ))
		thread.start()
		thread.join()

def popVelocityThreaded(control):
	if len(control.velocityQueue) == 0:
		pass
