#!/usr/bin/env python2

import sys

import roslib; roslib.load_manifest('dfDrone')
import rospy
import tf.transformations
from geometry_msgs.msg import Twist

def main(argv=None):
	if argv is None:
		argv = sys.argv

	rospy.init_node('move')
	p = rospy.Publisher('cmd_vel', Twist)

	twist = Twist()
	twist.linear.x = 1

	rospy.loginfo("about to move forward")
	for i in range(30):
		p.publish(twist)
		rospy.sleep(1)
		rospy.loginfo("moving")


	twist = Twist()
	rospy.loginfo("stopping")
	p.publish(twist)

if __name__ == "__main__":
	sys.exit(main())
