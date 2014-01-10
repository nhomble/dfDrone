#include <iostream>
#include <stdio.h>

#include "ros/ros.h"

class Twist {
	ros::NodeHandle n;
	ros::Publisher pub;
	
	public:
		void sendMessage(int x, int y){

		} 
};

Twist::Twist(){
	pub = n.advertise<std_msgs::cmd_vel>("twist", 100);
}

extern "C" {
	Twist* Twist_new(){
		return new Twist();
	}
	void Twist_sendMessage(Twist* twist, int x, int y){
		twist->sendMessage(x, y);
	}
}
