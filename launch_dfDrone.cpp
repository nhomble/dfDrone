#include <stdlib.h>
#include <stdio.h>

#include <ros/console.h>

int main(){
	ROS_DEBUG("Starting dfDrone");
	system("./dfDrone.py kinect");
	ROS_DEBUG("Ending program... should I have looped?");
	return 0;	
}
