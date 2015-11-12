/****************************************************************************
 *
 *   Copyright (c) 2015 Crossline Drone Project Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name CLDrone nor Crossline Drone nor the names of its c
 *    ontributors may be used to endorse or promote products derived from 
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <keyboard/Key.h>
#include "std_msgs/Empty.h"
#include <PIDController.h>


ros::Publisher CommandOutPub,TakeOffPub,LandPub,ResetPub;
geometry_msgs::Twist vs;
std_msgs::Empty EmptyMsg;
bool isMarkerTracking;

PIDController pid;
ros::Time last_time;

void sendCommand(const keyboard::Key &key)
{

  switch(key.code)
  {
  	case 'w':
  	{
  		ROS_INFO("Take off");
  		vs.angular.x=0.0;
        vs.angular.y=0.0;
        vs.angular.z=0.0;
        vs.linear.x=0.0;
        vs.linear.y=0.0;
        vs.linear.z=0.0;
  		TakeOffPub.publish(EmptyMsg);
  		break;
  	}
  	case 's':
  	{
  		ROS_INFO("Landing");
  		vs.angular.x=0.0;
        vs.angular.y=0.0;
        vs.angular.z=0.0;
        vs.linear.x=0.0;
        vs.linear.y=0.0;
        vs.linear.z=0.0;
  		LandPub.publish(EmptyMsg);
  		break;
  	}
    case 'a':
    {
    	ROS_INFO("Reset");
    	vs.angular.x=0.0;
        vs.angular.y=0.0;
        vs.angular.z=0.0;
        vs.linear.x=0.0;
        vs.linear.y=0.0;
        vs.linear.z=0.0;
    	ResetPub.publish(EmptyMsg);
    	break;
    }
    case 'i':
    {
    	vs.linear.x = 0.1;
    	CommandOutPub.publish(vs);
    	break;
    }
    case 'k':
    {
    	vs.linear.x = -0.1;
    	CommandOutPub.publish(vs);
    	break;

    }
    case 'j':
    {
    	vs.linear.y = 0.1;
    	CommandOutPub.publish(vs);
    	break;
    }
    case 'l':
    {
    	vs.linear.y = -0.1;
    	CommandOutPub.publish(vs);
    	break;
    }
    case 'u':
    {
    	vs.linear.z = 0.1;
    	CommandOutPub.publish(vs);
    	break;
    }
    case 'o':
    {
    	vs.linear.z = -0.1;
    	CommandOutPub.publish(vs);
    	break;
    }
    case 'x':
    {
    	vs.angular.x=0.0;
        vs.angular.y=0.0;
        vs.angular.z=0.0;
        vs.linear.x=0.0;
        vs.linear.y=0.0;
        vs.linear.z=0.0;
        CommandOutPub.publish(vs);
        break;
    }
    case 'd':
    {
    	ROS_INFO("Marker tracking");
    	isMarkerTracking = true;
    	break;
    }
    case 'c':
    {
    	ROS_INFO("Quit Marker tracking");
    	isMarkerTracking = false;
    	break;
    }


  }


}

void markerPoseReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{
   
   geometry_msgs::PoseStamped dronePose = *msg;
   geometry_msgs::Pose markerPose;
   // Front camera tracking
   markerPose.position.x = dronePose.pose.position.z;
   markerPose.position.z = -dronePose.pose.position.y;
   markerPose.position.y = -dronePose.pose.position.x;
   
   Eigen::Vector3d current = Eigen::Vector3d(markerPose.position.x,markerPose.position.y,markerPose.position.z);
   Eigen::Vector3d goal = Eigen::Vector3d(1,0,0);


   tf::vectorEigenToMsg(pid.compute_linvel_effort(goal, current, last_time), vs.linear);

   vs.linear.x = -vs.linear.x;
   vs.linear.y = -vs.linear.y;
   vs.linear.z = -vs.linear.z;
 
  
  if(isMarkerTracking)   
  {
	CommandOutPub.publish(vs);

  }                   
  
  last_time = ros::Time::now();

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_marker_tracker_node");
  ros::NodeHandle nodeHandle;

  //Publisher
  CommandOutPub = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  TakeOffPub = nodeHandle.advertise<std_msgs::Empty>("ardrone/takeoff",10);
  LandPub = nodeHandle.advertise<std_msgs::Empty>("ardrone/land",10);
  ResetPub = nodeHandle.advertise<std_msgs::Empty>("ardrone/reset",10);
  ros::Subscriber markerPoseSubscriber = nodeHandle.subscribe("/aruco_single/pose",10,markerPoseReceived);
  ros::Subscriber commandSubscriber = nodeHandle.subscribe("/keyboard/keydown",1,sendCommand);
  
  isMarkerTracking = false;
  last_time = ros::Time::now();

  // Setup of the PID controllers
   pid.setup_linvel_pid(0.4, 0.05, 0.12, 0.1, -0.1);


  ros::spin();

  return 0;

}