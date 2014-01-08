#!/usr/bin/env python2

import sys

from ctypes import cdll

class Control(object):
	def __init__(self):
			
		try:
			open('./libtwist', 'r')
			pass
		except IOError:
			print("where is libtwist?, make sure the file is compiled and in the same directory")
		self.lib = cdll.LoadLibrary('./libtwist')
	
	def addToCoordinateQueue(self, delX, delY):
		pass
