#!/bin/env python2

import sys

import SimpleCV
import freenect

import droneDetect
import twist

def main(argv=None):
	if argv is None:
		argv = sys.argv
	
	cam = SimpleCV.Camera()
	disp = SimpleCV.Display()

	detector = droneDetect.Detector(cam.getImage())
	control = twist.Control() 

	# use the display for debugging purposes
	# should check whether Ros::Ok()?
	while disp.isNotDone():
		img = cam.getImage()
		depth = None
		# ask our detector for new coordinates 
		valid, delX, delY = detector.process(img, depth)
		if valid is True:
			# add the new coordinates to the controller to process
			control.addToCoordinateQueue(delX, delY)

if __name__ == "__main__":
	sys.exit(main())
