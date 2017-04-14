#pragma once

/**
 * ROS standard lenght unit : meter [REP 103]
 *
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "DCMotor.h"

//////////////////////////////////////////////////////////////////

void ddr_callback(const geometry_msgs::Twist&);

//////////////////////////////////////////////////////////////////

const boolean LEFT PROGMEM = false;
const boolean RIGHT PROGMEM = true;

ros::NodeHandle nh;

std_msgs::Float32 angle_left;
std_msgs::Float32 angle_right;

ros::Publisher pub_left("/encoder/left", &angle_left);
ros::Publisher pub_right("/encoder/right", &angle_right);

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel_mux/input/teleop", ddr_callback);

//////////////////////////////////////////////////////////////////

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

	void Move(const double, const double);
	void UpdatePhysicalParameters(float, float);
	float GetEncoderAngle(boolean);
	void SendAngles();
	void Stop();
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
	(DCMotor *motor_left, DCMotor *motor_right, double rad, double dist) {

	this->motor_left = motor_left;
	this->motor_right = motor_right;
	this->wheel_radius = rad;
	this->wheel_distance = dist;
}

//////////////////////////////////////////////////////////////////

void DifferentialDriveRobot::Move(const double lin, const double ang) {

	double u_r = (lin + ang * this->wheel_distance/2.0) / this->wheel_radius;
	double u_l = (lin - ang * this->wheel_distance/2.0) / this->wheel_radius;

	INT_PWM right_map = 
		map(static_cast<INT_PWM>(u_r), 0, this->bound_right, 0, MAX_VALUE);
	
	INT_PWM left_map = 
		map(static_cast<INT_PWM>(u_l), 0, this->bound_left, 0, MAX_VALUE);


	u_r? motor_right->CW(right_map) : motor_right->CCW(right_map);
	u_l? motor_left->CCW(left_map) : motor_left->CW(left_map);
}

void DifferentialDriveRobot::UpdatePhysicalParameters(float max_speed, float max_turn) {
	
	this->max_speed = max_speed;
	this->max_turn = max_turn;

	this->bound_right = 
		(this->max_speed + this->max_turn * this->wheel_distance/2.0) / this->wheel_radius;
	this->bound_left = 
		(this->max_speed - this->max_turn * this->wheel_distance/2.0) / this->wheel_radius;
}

void DifferentialDriveRobot::Stop() {

	motor_left->Stop();
	motor_right->Stop();
}

float DifferentialDriveRobot::GetEncoderAngle(boolean motor) {

	if (motor == LEFT) {
		return motor_left->GetEncoderAngle();
	} else if(motor == RIGHT) {
		return motor_right->GetEncoderAngle();
	}
}

void DifferentialDriveRobot::SendAngles() {

	angle_left.data = 
		DifferentialDriveRobot::GetEncoderAngle(LEFT);
	angle_right.data = 
		DifferentialDriveRobot::GetEncoderAngle(RIGHT);

	pub_left.publish(&angle_left);
	pub_right.publish(&angle_right);
}