#pragma once

/**
 * ROS standard lenght unit : meter [REP 103]
 *
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "DCMotor.h"

//////////////////////////////////////////////////////////////////

const boolean LEFT PROGMEM = false;
const boolean RIGHT PROGMEM = true;

//////////////////////////////////////////////////////////////////

class DifferentialDriveRobot : public ActionBridge
{
	DCMotor *motor_left;
	DCMotor *motor_right;

	std_msgs::Float32 angle;

	double wheel_radius;
	double wheel_distance;
	double bound_right;
	double bound_left;
	float max_speed;
	float max_turn;

public:
	DifferentialDriveRobot();
	DifferentialDriveRobot(DCMotor*, DCMotor*);
	DifferentialDriveRobot(DCMotor*, DCMotor*, char*, char*);
	DifferentialDriveRobot(DCMotor*, DCMotor*, double, double);
	DifferentialDriveRobot(DCMotor*, DCMotor*, double, double, char*, char*);

	void ddr_callback(const geometry_msgs::Twist&);

	void FeedbackLeftCb(const std_msgs::UInt16 &);
	void FeedbackRightCb(const std_msgs::UInt16 &);

	void Move(const double, const double);
	void UpdatePhysicalParameters(float, float);
	float GetEncoderAngle(boolean);
	void SendAngles();
	void Stop();

	ros::NodeHandle nh;
	ros::Publisher pub_left;
	ros::Publisher pub_right;
	ros::Subscriber<geometry_msgs::Twist, DifferentialDriveRobot> sub;
};

DifferentialDriveRobot::DifferentialDriveRobot()
	: DifferentialDriveRobot::DifferentialDriveRobot(
		new DCMotor(IN1,IN2), 
		new DCMotor(IN3,IN4)) {}

DifferentialDriveRobot::DifferentialDriveRobot
	(DCMotor *motor_left, DCMotor *motor_right)
	: DifferentialDriveRobot::DifferentialDriveRobot(
		motor_left, motor_right, 0.032, 0.1) {}

DifferentialDriveRobot::DifferentialDriveRobot
	(DCMotor *motor_left, DCMotor *motor_right, double rad, double dist)
	: DifferentialDriveRobot::DifferentialDriveRobot(
		motor_left, motor_right, rad, dist, "/encoder/left", "/encoder/right") {}

DifferentialDriveRobot::DifferentialDriveRobot
	(DCMotor *motor_left, DCMotor *motor_right, char *topic_left, char *topic_right)
	: DifferentialDriveRobot::DifferentialDriveRobot(
		motor_left, motor_right, 0.032, 0.1, "/encoder/left", "/encoder/right") {}

DifferentialDriveRobot::DifferentialDriveRobot
	(DCMotor *motor_left, DCMotor *motor_right, double rad, double dist, char* topic_left, char* topic_right)
	: sub("/cmd_vel_mux/input/teleop", &DifferentialDriveRobot::ddr_callback, this),
	  pub_left(topic_left, &angle),
	  pub_right(topic_right, &angle),
	  ActionBridge(nh) {

	this->motor_left = motor_left;
	this->motor_right = motor_right;
	this->wheel_radius = rad;
	this->wheel_distance = dist;

	nh.initNode();

	nh.subscribe(sub);
	nh.advertise(pub_left);
	nh.advertise(pub_right);
}

//////////////////////////////////////////////////////////////////

void DifferentialDriveRobot::Move(const double lin, const double ang) {

	double u_r = (lin + ang * this->wheel_distance/2.0) / this->wheel_radius;
	double u_l = (lin - ang * this->wheel_distance/2.0) / this->wheel_radius;

	INT_PWM right_map = 
		map(static_cast<INT_PWM>(u_r), 0, this->bound_right, 0, MAX_VALUE);
	
	INT_PWM left_map = 
		map(static_cast<INT_PWM>(u_l), 0, this->bound_left, 0, MAX_VALUE);

	ActionBridge::SendGoal(right_map);
	ActionBridge::SendGoal(left_map);
	
	//u_r? motor_right->CW(right_map) : motor_right->CCW(right_map);
	//u_l? motor_left->CCW(left_map) : motor_left->CW(left_map);
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

	angle.data = DifferentialDriveRobot::GetEncoderAngle(LEFT);
	pub_left.publish(&angle);

	angle.data = DifferentialDriveRobot::GetEncoderAngle(RIGHT);
	pub_right.publish(&angle);
}

//////////////////////////////////////////////////////////////////

void DifferentialDriveRobot::ddr_callback(const geometry_msgs::Twist& msg_motor) {

  DifferentialDriveRobot::Move(msg_motor.linear.x, msg_motor.angular.z);
}

void DifferentialDriveRobot::FeedbackLeftCb(const std_msgs::UInt16 &msg) {

	msg.data? motor_left->CW(msg.data) : motor_left->CCW(msg.data);
}

void DifferentialDriveRobot::FeedbackRightCb(const std_msgs::UInt16 &msg) {

	msg.data? motor_right->CW(msg.data) : motor_right->CCW(msg.data);
}