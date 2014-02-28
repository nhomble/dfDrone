#!/usr/bin/env python2

import sys

import SimpleCV

import detectDrone

def main(argv=None):
	if argv is None:
		argv = sys.argv
	if len(argv) != 3:
		print(str(argv[0]) + " positive/negative imgPath")
	isPositive = False;
	if argv[1] == "positive":
		isPositive = True;

	disp = SimpleCV.Display()
	img = SimpleCV.Image(str(argv[2]))

	detector = detectDrone.Detector(True)
	hasDrone, centroid, z = detector.hasDrone(img, None)
	if isPositive is hasDrone:
		pass
	else:
		print("should be", isPositive, "but I got", hasDrone, argv[2])

if __name__ == "__main__":
	sys.exit(main())
