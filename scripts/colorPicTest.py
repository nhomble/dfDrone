#!/usr/bin/env python2

import os
import sys
import SimpleCV
import time

import detectDrone
import messageDrone

TIME = 1

def main(argv=None):
	if argv is None:
		argv = sys.argv

	detector = detectDrone.Detector()

	total = 0
	count = 0
	verbose = False
	if len(argv) > 2 and argv[2] == "verbose":
		verbose = True

	if argv[1] == "positive":
		for fn in os.listdir('testing/positive'):
			total += 1
			img = SimpleCV.Image('testing/positive/' + fn)
			message = detector.process(img, None)
			if message.isPresent is False:
				count += 1
				if verbose:
					img.show()
					time.sleep(TIME)
	else:	
		for fn in os.listdir('testing/negative'):
			total += 1
			img = SimpleCV.Image('testing/negative/' + fn)
			message = detector.process(img, None)
			if message.isPresent is True:
				count += 1
				if verbose:
					img.show()
					time.sleep(TIME)

	percent = float(count) / float(total)
	print("percent bad " + str(percent))

if __name__ == "__main__":
	sys.exit(main())
