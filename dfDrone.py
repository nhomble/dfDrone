#!/usr/bin/env python2

import sys

import SimpleCV
import freenect

import droneDetect

def main(argv=None):
	if argv is None:
		arv = sys.argv
	
	cam = SimpleCV.Camera()
	disp = SimpleCV.Display()

	detector = droneDetect.Detector(cam.getImage())

	# use the display for debugging purposes
	# should check whether Ros::Ok()?
	while disp.isNotDone():
		img = cam.getImage()
		# ask our detector if there is a drone in the image
		if detector.process(img):
			pass

if __name__ == "__main__":
	sys.exit(main())
