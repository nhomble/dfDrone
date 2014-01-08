#!/usr/bin/env python2

import sys

import SimpleCV

import droneDetect

def main(argv=None):
	if argv is None:
		argv = sys.argv
	if len(argv) != 3:
		print(str(argv[0]) + " positive/negative imgPath")
	isPositive = False;
	if argv[1] == "positive":
		isPositive = True;
	img = SimpleCV.Image("./" + str(argv[2]))
	
	detector = droneDetect.Detector(img, False)
	if isPositive == detector.hasDrone(img, None):
		pass
	else:
		print("./" + argv[2])

if __name__ == "__main__":
	sys.exit(main())
