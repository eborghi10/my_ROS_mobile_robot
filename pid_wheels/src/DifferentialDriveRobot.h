#pragma once

/**
 * ROS standard lenght unit : meter [REP 103]
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

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
	double boundRight;
	double boundLeft;
	float maxSpeed;
	float maxTurn;

	INT_PWM map(INT_PWM, INT_PWM, INT_PWM, INT_PWM, INT_PWM);

public:

	DifferentialDriveRobot();

	void keyboardCb(const geometry_msgs::Twist&);

	ros::NodeHandle nh;
	ros::Subscriber sub;
};

//////////////////////////////////////////////////////////////////

DifferentialDriveRobot::DifferentialDriveRobot()
	: wheelRadius(0.032), wheelDistance(0.1), 
	  maxSpeed(0.2), maxTurn(1.0)
	  {
	  	boundRight = (maxSpeed + maxTurn * wheelDistance/2.0) / wheelRadius;
	  	boundLeft  = (maxSpeed - maxTurn * wheelDistance/2.0) / wheelRadius;

	  	sub = nh.subscribe("/cmd_vel_mux/input/teleop", 1, &DifferentialDriveRobot::keyboardCb, this);
	  }

//////////////////////////////////////////////////////////////////

INT_PWM DifferentialDriveRobot::map(
	INT_PWM value, INT_PWM fromLow, INT_PWM fromHigh, INT_PWM toLow, INT_PWM toHigh)
{
	return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

//////////////////////////////////////////////////////////////////

void DifferentialDriveRobot::keyboardCb(const geometry_msgs::Twist& msg) {

	double lin = msg.linear.x;
	double ang = msg.angular.z;

  	double u_r = (lin + ang * wheelDistance/2.0) / wheelRadius;
	double u_l = (lin - ang * wheelDistance/2.0) / wheelRadius;

	INT_PWM right_map = 
		map(static_cast<INT_PWM>(u_r), 0, boundRight, 0, MAX_VALUE);
	
	INT_PWM left_map = 
		map(static_cast<INT_PWM>(u_l), 0, boundLeft, 0, MAX_VALUE);
}