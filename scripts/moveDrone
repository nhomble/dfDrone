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
with threads I hope to parallelize the queue of velocity vectors
'''

# use when accessing velocityQueue
queueLock = threading.Lock()

# this is the class that dfDrone sees and communicates with
# everything else is internal
class Control(object):
	def __init__(self):
		self.velocityQueue = []

		rospy.init_node('move_dfDrone')
		self.pub = rospy.Publisher('cmd_vel', Twist)

		# start a thread that does not die
		thread = popVelocityThread(self)	
		thread.start()
		thread.join()
	
	# spawn thread to add tuple to velocityQueue
	def addToVelocityQueue(self, delX, delY, delZ, delT):
		thread = addToQueueThread(self, delX, delY, delZ, delT)
		thread.start()
		thread.join()

	# TODO
	# we could make it smarted, but let's just spin now
	def randomWalk(self):
		rospy.loginfo("moving aimlessly")
		twist = Twist()
		twist.linear.x = 1
		twist.linear.y = 1
		time.sleep(1)

		stopMoving()

	def stopMoving(self):
		rospy.loginfo("stopping")
		twist = Twist()
		self.pub.publish(twist)

# spawned by control, I just keep pestering
# the queue trying to get a velocity vector to work
# with
class popVelocityThread(threading.Thread):
	def __init__(self, control):
		threading.Thread.__init__(self, control)
		self.control = control

	def run(self):
		while True:
			popVelocityThreaded(self.control)

# called by control to put new data into the queue
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

# function given to thread
def addToVelocityQueueThreaded(control, velocity):
	with queueLock:
		control.velocityQueue.append(velocity)

# function given to thread
def popVelocityThreaded(control):
	if len(control.velocityQueue) == 0:
		return

	with queueLock:
		localData = threading.local()
		localData.velTuple = control.velocityQueue.pop()

	startTime = time.gmtime()
	# rotate
	# determine arc length (TODO maybe use?)
	radians = math.atan2(x, y)
	arcLength = math.pi * math.pow(localData.velTuple[2], 2)
	arcLength = arcLength * radians / (2 * math.pi)

	while localData.velTuple[3] > time.gmtime() - startTime:
		rospy.loginfo("turning")
		twist = Twist()
		if localData.velTuple[2] < 0:
			twist.angular.x = 1
		elif localData.velTuple[2] > 0:
			twist.angular.x = -1
		else:
			break
		control.pub.publish(twist)
		time.sleep(.5)

	startTime = time.gmtime()
	# move closer
	while localData.velTuple[3] > time.gmtime() - startTime:
		rospy.loginfo("moving forward")
		twist = Twist()
		twist.linear.x = 1
		control.pub.publish(twist)
		time.sleep(.5)
		stopMoving()
