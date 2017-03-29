#pragma once

/**
 * ROS standard lenght unit : meter [REP 103]
 *
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "DCMotor.h"

//////////////////////////////////////////////////////////////////

sensor_msgs::JointState msg_angle;
ros::Publisher pub("/encoder", &msg_angle);

void ddr_callback(const geometry_msgs::Twist& msg_motor);

ros::Subscriber<geometry_msgs::Twist> sub(
  "/cmd_vel_mux/input/teleop", 
  ddr_callback);

ros::Time prevTime(0,0);
double dT;
double prevPosition[] = {0, 0};

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
	int Move(const double, const double);
	void UpdatePhysicalParameters(float, float);
	char* GetEncoderTopicName(uint8_t);
	double GetEncoderAngle(uint8_t);
	void Stop();

	ros::NodeHandle nh;
};

DifferentialDriveRobot::DifferentialDriveRobot()
	: DifferentialDriveRobot::DifferentialDriveRobot(
		new DCMotor(4,5), new DCMotor(6,7), 0.032, 0.1) {}

DifferentialDriveRobot::DifferentialDriveRobot
	(DCMotor *motor_left, DCMotor *motor_right)
	: wheel_radius(0.032), wheel_distance(0.1) {
	this->motor_left = motor_left;
	this->motor_right = motor_right;
}

DifferentialDriveRobot::DifferentialDriveRobot
	(DCMotor *motor_left, DCMotor *motor_right, double rad, double dist) {
	this->motor_left = motor_left;
	this->motor_right = motor_right;
	this->wheel_radius = rad;
	this->wheel_distance = dist;

	nh.initNode();

	nh.subscribe(sub);
	nh.advertise(pub);
}

//////////////////////////////////////////////////////////////////

int DifferentialDriveRobot::Move(const double lin, const double ang) {

	double u_r = (lin + ang * this->wheel_distance/2.0) / this->wheel_radius;
	double u_l = (lin - ang * this->wheel_distance/2.0) / this->wheel_radius;

	int var_test = map(static_cast<int>(u_r), 0, this->bound_right, 0, 255);

	motor_right->PWM(var_test);
	
	motor_left->PWM(map(static_cast<int>(u_l), 0, this->bound_left, 0, 255));

	return var_test;
}

void DifferentialDriveRobot::UpdatePhysicalParameters(float max_speed, float max_turn) {
	
	this->max_speed = max_speed;
	this->max_turn = max_turn;

	this->bound_right = 
		(this->max_speed + this->max_turn * this->wheel_distance/2.0) / this->wheel_radius;
	this->bound_left = 
		(this->max_speed - this->max_turn * this->wheel_distance/2.0) / this->wheel_radius;
}

char* DifferentialDriveRobot::GetEncoderTopicName(uint8_t position) {

	if (position == LEFT) {
		return motor_left->GetEncoderTopicName();
	} else if(position == RIGHT) {
		return motor_right->GetEncoderTopicName();
	} else {
		char* topic_name = "encoder";
		return topic_name;
	}
}

double DifferentialDriveRobot::GetEncoderAngle(uint8_t position) {

	if (position == LEFT) {
		return motor_left->GetEncoderAngle();
	} else if(position == RIGHT) {
		return motor_right->GetEncoderAngle();
	}
}

void DifferentialDriveRobot::Stop() {

	motor_left->Stop();
	motor_right->Stop();
}