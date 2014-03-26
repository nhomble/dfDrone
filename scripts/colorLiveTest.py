#!/usr/bin/env python2

import sys
import time
import SimpleCV
import detectDrone
import messageDrone

def main(argv=None):
	if argv is None:
		argv = sys.argv
	cam = SimpleCV.Camera()
	disp = SimpleCV.Display()

	detector = detectDrone.Detector(True)

	while disp.isNotDone():
		img = cam.getImage()

		message = detector.process(img, None)
		if message.isPresent is True:
			print(message.x, message.y)

if __name__ == "__main__":
	sys.exit(main())
