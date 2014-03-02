#!/usr/bin/env python2

import sys
import SimpleCV
import detectDrone
import messageDrone

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
	message = detector.process(img, None)
	if message.isPresent is isPositive:
		pass
	else:
		print("should be", isPositive, "but I got", message.isPresent, argv[2])

if __name__ == "__main__":
	sys.exit(main())
