#include <iostream>
#include <stdio.h>

#include "ros/ros.h"

class Twist {
	ros::NodeHandle n;
	ros::Publisher pub;

	public:
		Twist ();
		// need to create twist message and send via ROS
		void sendMessage(int x, int y){

		} 
} twist;

// need to initialize the publisher
Twist::Twist(){
}

extern "C" {
	Twist* Twist_new(){
		return new Twist();
	}
	void Twist_sendMessage(Twist* twist, int x, int y){
		twist->sendMessage(x, y);
	}
}
