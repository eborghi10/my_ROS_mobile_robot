#pragma once

/**
 * ROS standard lenght unit : meter [REP 103]
 *
 */
/*
#if defined(__AVR_ATmega32U4__)
  #define USE_USBCON
#endif
*/
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "DCMotor.h"

class DifferentialDriveRobot
{
	DCMotor *motor_left;
	DCMotor *motor_right;
	double wheel_radius;
	double wheel_distance;
	double bound_right;
	double bound_left;
	float max_speed;
	float max_turn;
public:
	DifferentialDriveRobot();
	DifferentialDriveRobot(DCMotor*, DCMotor*);
	DifferentialDriveRobot(DCMotor*, DCMotor*, double, double);
	void move(const double, const double);
	void updateParameters(float, float);

	void ddr_callback(const geometry_msgs::Twist&);

	ros::NodeHandle nh;
	ros::Subscriber <geometry_msgs::Twist, DifferentialDriveRobot> sub;
};

DifferentialDriveRobot::DifferentialDriveRobot()
	: DifferentialDriveRobot::DifferentialDriveRobot(
		new DCMotor(IN1,IN2), 
		new DCMotor(IN3,IN4), 0.032, 0.1) {}

DifferentialDriveRobot::DifferentialDriveRobot
	(DCMotor *motor_left, DCMotor *motor_right)
	: DifferentialDriveRobot::DifferentialDriveRobot(
		motor_left, motor_right, 0.032, 0.1) {}

DifferentialDriveRobot::DifferentialDriveRobot
	(DCMotor *motor_left, DCMotor *motor_right, double rad, double dist)
	: sub("/cmd_vel_mux/input/teleop", &DifferentialDriveRobot::ddr_callback, this) {
	this->motor_left = motor_left;
	this->motor_right = motor_right;
	this->wheel_radius = rad;
	this->wheel_distance = dist;

	nh.initNode();
  	nh.subscribe(sub);
}

void DifferentialDriveRobot::move(const double lin, const double ang) {

	/**
	 * NOTE ON MATHEMATICAL MODEL: 
	 * 
	 * ang < 0: moves CW (u_l > u_r)
	 *
	 * To change this, you have to change the sign of the second term
	 *
	 */

	double u_r = (lin + ang * this->wheel_distance/2.0) / this->wheel_radius;
	double u_l = (lin - ang * this->wheel_distance/2.0) / this->wheel_radius;

	if (u_r > 0) {
		

		motor_right->CW(
			map(static_cast<INT_PWM>(u_r), 0, this->bound_right, 0, MAX_VALUE));
	} else {

		motor_right->CCW(
			map(static_cast<INT_PWM>(-u_r), 0, this->bound_right, 0, MAX_VALUE));
	}

	if (u_l > 0) {
		
		motor_left->CCW(
			map(static_cast<INT_PWM>(u_l), 0, this->bound_left, 0, MAX_VALUE));
	} else {

		motor_left->CW(
			map(static_cast<INT_PWM>(-u_l), 0, this->bound_left, 0, MAX_VALUE));
	}
}

void DifferentialDriveRobot::updateParameters(float max_speed, float max_turn) {
	
	this->max_speed = max_speed;
	this->max_turn = max_turn;
	this->bound_right = 
		(this->max_speed + this->max_turn * this->wheel_distance/2.0) / this->wheel_radius;
	this->bound_left = 
		(this->max_speed - this->max_turn * this->wheel_distance/2.0) / this->wheel_radius;
}

void DifferentialDriveRobot::ddr_callback(const geometry_msgs::Twist& msg) {

	DifferentialDriveRobot::move(msg.linear.x, msg.angular.z);
	
//	char* str1 = "";
//	dtostrf(lin, 2, 2, str1);
//	nh.loginfo(str1);
/*
	char* str = "";
	snprintf(str,sizeof(int)*2,"%d",var);
	nh.loginfo(str);
*/
	//  delay(1);
}