/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <string>
#include <stdio.h>
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include "ros/ros.h"
#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "dynamic_reconfigure/server.h"
#include "turtlebot_follower/ChangeState.h"
#include "turtlebot_follower/FollowerConfig.h"

#include <visualization_msgs/Marker.h>

#include "std_msgs/Float64MultiArray.h"

namespace turtlebot_follower
{
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//* The turtlebot follower nodelet.
/**
 * The turtlebot follower nodelet. Subscribes to point clouds
 * from the 3dsensor, processes them, and publishes command vel
 * messages.
 */
class TurtlebotFollower : public nodelet::Nodelet
{
public:
  /*!
   * @brief The constructor for the follower.
   * Constructor for the follower.
   */
  TurtlebotFollower() : min_y_(0.1), max_y_(0.5),
                        min_x_(-0.2), max_x_(0.2),
			min_x_l_(-0.3), max_x_l_(-0.2),
			min_x_r_(0.2), max_x_r_(0.3),
                        max_z_(1.0), goal_z_(0.6),
                        z_scale_(1.0), x_scale_(5.0),
			fresh(0), turn_dir(0)
  {
    //max_z_ original value is 0.8
  }

  ~TurtlebotFollower()
  {
    delete config_srv_;
  }

private:
  double min_y_; /**< The minimum y position of the points in the box. */
  double max_y_; /**< The maximum y position of the points in the box. */

  double min_x_; /**< The minimum x position of the points in the box. */
  double max_x_; /**< The maximum x position of the points in the box. */

  //Defines the left and right regions for alternate centroids
  double min_x_l_;
  double max_x_l_;
  double min_x_r_;
  double max_x_r_;

  double max_z_; /**< The maximum z position of the points in the box. */
  double goal_z_; /**< The distance away from the robot to hold the centroid */
  double z_scale_; /**< The scaling factor for translational robot speed */
  double x_scale_; /**< The scaling factor for rotational robot speed */
  bool   enabled_; /**< Enable/disable following; just prevents motor commands */

  int    fresh; //Is the value of turn_dir recent?
  int    turn_dir; //-1 means left. 0 means straight. 1 means right.

  // Service for start/stop following
  ros::ServiceServer switch_srv_;

  // Dynamic reconfigure server
  dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>* config_srv_;

  /*!
   * @brief OnInit method from node handle.
   * OnInit method from node handle. Sets up the parameters
   * and topics.
   */
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("min_y", min_y_);
    private_nh.getParam("max_y", max_y_);
    private_nh.getParam("min_x", min_x_);
    private_nh.getParam("max_x", max_x_);
    private_nh.getParam("max_z", max_z_);
    private_nh.getParam("goal_z", goal_z_);
    private_nh.getParam("z_scale", z_scale_);
    private_nh.getParam("x_scale", x_scale_);
    private_nh.getParam("enabled", enabled_);

    cmdpub_ = private_nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    markerpub_ = private_nh.advertise<visualization_msgs::Marker>("marker",1);
    bboxpub_ = private_nh.advertise<visualization_msgs::Marker>("bbox",1);
    //sub_= nh.subscribe<PointCloud>("depth/points", 1, &TurtlebotFollower::cloudcb, this);
    sub_ = nh.subscribe<Float64MultiArray>("centerDrone", 1, &TurtlebotFollower::move_callback, this);

    //Add for debug messages
    debugpub_ = nh.advertise<std_msgs::String>("debug", 1);


    switch_srv_ = private_nh.advertiseService("change_state", &TurtlebotFollower::changeModeSrvCb, this);

    config_srv_ = new dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>(private_nh);
    dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>::CallbackType f =
        boost::bind(&TurtlebotFollower::reconfigure, this, _1, _2);
    config_srv_->setCallback(f);
  }

  void reconfigure(turtlebot_follower::FollowerConfig &config, uint32_t level)
  {
    min_y_ = config.min_y;
    max_y_ = config.max_y;
    min_x_ = config.min_x;
    max_x_ = config.max_x;
    max_z_ = config.max_z;
    goal_z_ = config.goal_z;
    z_scale_ = config.z_scale;
    x_scale_ = config.x_scale;
  }

// reviewed the actual (vanilla) turtlebot follower code
  void move_callback(const std_msgs::Float64MultiArray message)
  {
	x = message[0];
	y = message[1];
	z = message[2];
      	ROS_DEBUG("Centroid at %f %f %f", x, y, z);
      	publishMarker(x, y, z);

	if (enabled_){
  	      geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
  	      cmd->linear.x = (z - goal_z_) * z_scale_;

  	      cmd->angular.z = -x * x_scale_;

  	      cmdpub_.publish(cmd);
  	}
  }

  /*!
   * @brief Callback for point clouds.
   * Callback for point clouds. Uses PCL to find the centroid
   * of the points in a box in the center of the point cloud.
   * Publishes cmd_vel messages with the goal from the cloud.
   * @param cloud The point cloud message.
   */
  void cloudcb(const PointCloud::ConstPtr&  cloud)
  {
    //X,Y,Z of the centroid
    float x = 0.0;
    float x_l = 0.0;
    float x_r = 0.0;
    float y = 0.0;
    float z = 1e6;
    float z_l = 1e6;
    float z_r = 1e6;
    //Number of points observed
    unsigned int n = 0;
    unsigned int n_l = 0;
    unsigned int n_r = 0;
    //Strings for publishing
    std::stringstream ss;
    std_msgs::String str;
    //Iterate through all the points in the region and find the average of the position
    /*BOOST_FOREACH (const pcl::PointXYZ& pt, cloud->points)
    {
      //First, ensure that the point's position is valid. This must be done in a separate
      //if because we do not want to perform comparison on a nan value.
      if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
      {
	//Original code section. Modified for center region
        //Test to ensure the point is within the acceptable box.
        if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z < max_z_)
        {
          //Add the point to the totals
          x += pt.x;
          y += pt.y;
          z = std::min(z, pt.z);
          n++;
        }

	//Adding left region
	//Test to ensure the point is within the acceptable box.
        if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_l_ && pt.x > min_x_l_ && pt.z < max_z_)
        {
          //Add the point to the totals
          x_l += pt.x;
          y += pt.y;
          z_l = std::min(z_l, pt.z);
          n_l++;
        }

	//Adding right region
	//Test to ensure the point is within the acceptable box.
        if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_r_ && pt.x > min_x_r_ && pt.z < max_z_)
        {
          //Add the point to the totals
          x_r += pt.x;
          y += pt.y;
          z_r = std::min(z_r, pt.z);
          n_r++;
        }



      }
    }*/

    //If there are points, find the centroid and calculate the command goal.
    //If there are no points, simply publish a stop goal.
    if (n>4000)
    {
      x /= n;
      x_l /= n_l;
      x_r /= n_r;

      y /= n;

      //Original Follower Code. Modify to avoid.
      /*
      if(z > max_z_){
        ROS_DEBUG("No valid points detected, stopping the robot");
        if (enabled_)
        {
          cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
        }
        return;
      }
      */

      //Infinite turning...except it doesn't work. The turtlebot does not turn smoothly/continuously
      /*
      while(1){
	geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
	cmd->angular.z = 1.0;
	cmdpub_.publish(cmd);
      }
      */

      //Avoider Code
      //If the turtlebot sees nothing
      if(z > max_z_){
        ROS_DEBUG("Area is clear. Moving forward");
        if (enabled_)
        {
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

	//This works to prevent collisions with obstacles in the front
        cmd->linear.x = (z - goal_z_) * z_scale_;

	if(z_l > 1.1 * z_r){
	  cmd->angular.z = -0.6; //Go left if there is more space to the left
	    }
	else if(z_r > 1.1 * z_l){
	  cmd->angular.z = 0.6; //Go right if there is more space to the right
	    }
	else{
	  cmd->angular.z = 0; //Go straight if the space is similar
	}

        cmdpub_.publish(cmd);
        }
        return;
      }

      //From here, code is for when a wall is at most max_z_ in front of the turtlebot
      ROS_DEBUG("Centroid at %f %f %f with %d points", x, y, z, n);
      publishMarker(x, y, z);

      if (enabled_)
      {
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
        cmd->linear.x = (z - goal_z_) * z_scale_;

        cmd->angular.z = x * x_scale_;

        cmdpub_.publish(cmd);
      }
    }
    else
    {

      /*
      ROS_DEBUG("No points detected, stopping the robot");
      publishMarker(x, y, z);

      if (enabled_)
      {
        cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      }
      */

      if (enabled_)
	{
	  geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

	  //cmd->linear.x
	  cmd->linear.x = (z - goal_z_) * z_scale_;

	  if(z > 2.0 * goal_z_){
	    fresh = 0;
	  }

	  if(z_l > 1.1 * z_r){
	    cmd->angular.z = -0.6; //Go left if there is more space to the left                        
	  }
	  else if(z_r > 1.1 * z_l){
	    cmd->angular.z = 0.6; //Go right if there is more space to the right                       
	  }
	  //Original condition: else{}. For P1, use > z-goal_z_>1.5*goal_z_
	  else if((z - goal_z_)  > 1.5 * goal_z_){
	    cmd->angular.z = 0; //Go straight if no points are close by

	  }
	  
	  /**/

	  
	  // Proposal #1 for solving the square corner problem: Add simple history/predictor
	  // If the turtlebot is close to goal, begin to save data
	  //else if(z - goal_z_ >= 1.3 * goal_z_ && !fresh){
	  else if(!fresh){
	    if(z_l > 1.1 * z_r){
	      //Some value says go left
	      turn_dir = -1;
	      fresh = 1;
	    }
	    else if(z_r > 1.1 * z_l){
	      //Some value says go right
	      turn_dir = 1;
	      fresh = 1;
	    }
	    else{
	      //Nearly equal. dunno what to do.
	      //turn_dir = 0;
	    }
	    //
	    ss << "I predict " << turn_dir << std::endl;
	    str.data = ss.str();
	    debugpub_.publish(str);
	  }
	  else{
	    //Fix straight at wall problem.
	    cmd->linear.x = goal_z_ * z_scale_;
	    if(turn_dir == -1){
	      cmd->angular.z = -1.0;
	    }
	    else if(turn_dir == 1){
	      cmd->angular.z = 1.0;
	    }
	    else{
	      //Panic! I'm going right.
	      cmd->angular.z = 1.0;
	    }
	    //
	    ss << "I will turn " << turn_dir << std::endl;
	    str.data = ss.str();
	    debugpub_.publish(str);

	    turn_dir = 0;
	    fresh = 0;
	    
	  }
	  /**/
	  //std::printf("Current action is going forward %d, turning %d\n", cmd->linear.x, cmd->angular.z);
	  //std::cout << "Action is " << cmd->linear.x << "," << cmd->angular.z << std::endl;
	  //
	  cmdpub_.publish(cmd);
	}

    }

    publishBbox();
  }

  bool changeModeSrvCb(ChangeState::Request& request, ChangeState::Response& response)
  {
    if ((enabled_ == true) && (request.state == request.STOPPED))
    {
      ROS_INFO("Change mode service request: following stopped");
      cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      enabled_ = false;
    }
    else if ((enabled_ == false) && (request.state == request.FOLLOW))
    {
      ROS_INFO("Change mode service request: following (re)started");
      enabled_ = true;
    }

    response.result = response.OK;
    return true;
  }

  void publishMarker(double x,double y,double z)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    markerpub_.publish( marker );
  }

  void publishBbox()
  {
    double x = (min_x_ + max_x_)/2;
    double y = (min_y_ + max_y_)/2;
    double z = (0 + max_z_)/2;

    double scale_x = (max_x_ - x)*2;
    double scale_y = (max_y_ - y)*2;
    double scale_z = (max_z_ - z)*2;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = -y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    bboxpub_.publish( marker );
  }

  ros::Subscriber sub_;
  ros::Publisher cmdpub_;
  ros::Publisher markerpub_;
  ros::Publisher bboxpub_;

  //Add a Publisher for debug
  ros::Publisher debugpub_;
};

PLUGINLIB_DECLARE_CLASS(turtlebot_follower, TurtlebotFollower, turtlebot_follower::TurtlebotFollower, nodelet::Nodelet);

}
