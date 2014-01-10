#include <stdlib.h>
#include <stdio.h>
#include <sstream>

#include "ros/ros.h"

int main(int argc, char *argv[]){
//	ros::init(argc, argv, "dfDrone");

//	while(ros::ok()){
//		ROS_DEBUG("Starting dfDrone");
		system("./dfDrone.py kinect");
//		ROS_DEBUG("Ending program... going to loop");
//	}
	return 0;	
}
