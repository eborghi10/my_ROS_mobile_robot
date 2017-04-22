#pragma once

/**
 * ROS standard lenght unit : meter [REP 103]
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <robot_msgs/Arduino.h>

//////////////////////////////////////////////////////////////////

//#define _16BIT_PWM_

#ifdef _16BIT_PWM_
  #define INT_PWM   uint16_t
  const INT_PWM MAX_VALUE = 0xFFFF;
#else
  #define INT_PWM   uint8_t
  const INT_PWM MAX_VALUE = 0xFF;
#endif

//////////////////////////////////////////////////////////////////

class DifferentialDriveRobot
{
	double wheelRadius;
	double wheelDistance;
	//double boundRight;
	//double boundLeft;
	float maxSpeed;
	float maxTurn;

	ros::NodeHandle nh_;
	ros::Subscriber sub;
	ros::Publisher pub;

	INT_PWM map(INT_PWM, INT_PWM, INT_PWM, INT_PWM, INT_PWM);

	void keyboardCb(const geometry_msgs::Twist::ConstPtr&);

public:

	DifferentialDriveRobot(ros::NodeHandle&);
};

//////////////////////////////////////////////////////////////////

DifferentialDriveRobot::DifferentialDriveRobot(ros::NodeHandle& nh)
	: wheelRadius(0.032), wheelDistance(0.1), 
	  maxSpeed(0.2), maxTurn(1.0), 
	  nh_(nh)
	  {
	  	//boundRight = (maxSpeed + maxTurn * wheelDistance/2.0) / wheelRadius;
	  	//boundLeft  = (maxSpeed - maxTurn * wheelDistance/2.0) / wheelRadius;

	  	sub = nh_.subscribe("/cmd_vel_mux/input/teleop", 1, &DifferentialDriveRobot::keyboardCb, this);
	  	pub = nh_.advertise<robot_msgs::Arduino>("/wheel_velocities", 1);

	  	ROS_INFO("Differential Drive Robot initialized.");
	  }

//////////////////////////////////////////////////////////////////
/*
INT_PWM DifferentialDriveRobot::map(
	INT_PWM value, INT_PWM fromLow, INT_PWM fromHigh, INT_PWM toLow, INT_PWM toHigh)
{
	return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}
*/
//////////////////////////////////////////////////////////////////

void DifferentialDriveRobot::keyboardCb(const geometry_msgs::Twist::ConstPtr& kyb) 
{

	double lin = kyb->linear.x;
	double ang = kyb->angular.z;

	ROS_INFO("[lin, ang] : [%f, %f]", lin, ang);

	double u_l = (lin - ang * wheelDistance/2.0) / wheelRadius;
  	double u_r = (lin + ang * wheelDistance/2.0) / wheelRadius;

//	INT_PWM right_map = 
//		map(static_cast<INT_PWM>(u_r), 0, boundRight, 0, MAX_VALUE);

	robot_msgs::Arduino msg;

	msg.name = "left";
	msg.data = u_l;
	pub.publish(msg);

	msg.name = "right";
	msg.data = u_r;
	pub.publish(msg);

	ROS_INFO("[u_l, u_r] : [%f, %f]", u_l, u_r);
	
//	INT_PWM left_map = 
//		map(static_cast<INT_PWM>(u_l), 0, boundLeft, 0, MAX_VALUE);
}