#!/bin/env python2

import sys
import time
import SimpleCV
import droneDetect

def main(argv=None):
	if argv is None:
		argv = sys.argv
	cam = SimpleCV.Camera()
	disp = SimpleCV.Display()

	detector = droneDetect.Detector(cam.getImage(), False, True)

	while disp.isNotDone():
		img = cam.getImage()

		hasDrone, x, y, z, t = detector.process(img, None)
		if hasDrone is True:
			print(x, y, t)

if __name__ == "__main__":
	sys.exit(main())
