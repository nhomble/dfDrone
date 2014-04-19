#!/usr/bin/env python2

import os
import sys
import SimpleCV
import time

import detectDrone
import messageDrone

def main(argv=None):
	if argv is None:
		argv = sys.argv

	detector = detectDrone.Detector()
	if argv[1] == "positive":

		for fn in os.listdir('testing/positive'):
			img = SimpleCV.Image('testing/positive/' + fn)
			message = detector.process(img, None)
			if message.isPresent is False:
				img.show()
				time.sleep(2)
	else:	
		for fn in os.listdir('testing/negative'):
			img = SimpleCV.Image('testing/negative/' + fn)
			message = detector.process(img, None)
			if message.isPresent is True:
				img.show()
				time.sleep(2)

if __name__ == "__main__":
	sys.exit(main())
