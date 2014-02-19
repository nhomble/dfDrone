#!/bin/env python2

import sys

import SimpleCV
import freenect

import droneDetect
import twist

# TODO should use a better command line parser 
def main(argv=None):
	if argv is None:
		argv = sys.argv

	# by default use a regular camera
	disp = SimpleCV.Display()
	cam = SimpleCV.Camera()
	useKinect = False

	# but try to get the kinect
	if len(argv) > 1 and argv[1] is "kinect":
		try:
			cam = SimpleCV.Kinect()
			useKinect = True
		except:
			print("COULD NOT GET KINECT via SIMPLECV")
			useKinect = False
			try:
				cam = SimpleCV.Camera()
			except:
				print("CANNOT GET REGULAR CAMERA, ABORT")
				return


	detector = droneDetect.Detector(cam.getImage(), useKinect, False)
	control = twist.Control() 

	# use the display for debugging purposes
	# TODO check whether Ros::Ok()?
	while True:
		img = cam.getImage()
		depth = None

		if useKinect is True:
			depth = cam.getDepth()

		valid, delX, delY, delZ, delT = detector.process(img, depth)
		if valid is True:
			control.addToVelocityQueue(delX, delY, delZ, delT)
		else:
			control.randomWalk()
			detector().reCenter()

if __name__ == "__main__":
	sys.exit(main())
