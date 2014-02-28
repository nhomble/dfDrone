#!/usr/bin/env python2

import sys
import time
import SimpleCV
import detectDrone

def main(argv=None):
	if argv is None:
		argv = sys.argv
	cam = SimpleCV.Camera()
	disp = SimpleCV.Display()

	detector = detectDrone.Detector(True)

	while disp.isNotDone():
		img = cam.getImage()

		hasDrone, centroid, z = detector.process(img, None)
		if hasDrone is True:
			print(centroid)

if __name__ == "__main__":
	sys.exit(main())
