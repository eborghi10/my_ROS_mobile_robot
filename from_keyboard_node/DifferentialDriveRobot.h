#pragma once

/**
 * ROS standard lenght unit : meter [REP 103]
 *
 */

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
	int move(const double, const double);
	void updateParameters(float, float);
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
}

int DifferentialDriveRobot::move(const double lin, const double ang) {

	double u_r = (lin + ang * this->wheel_distance/2.0) / this->wheel_radius;
	double u_l = (lin - ang * this->wheel_distance/2.0) / this->wheel_radius;

	int var_test = map(static_cast<int>(u_r), 0, this->bound_right, 0, 255);

	motor_right->PWM(
		var_test);
	motor_left->PWM(
		map(static_cast<int>(u_l), 0, this->bound_left, 0, 255));

	return var_test;
}

void DifferentialDriveRobot::updateParameters(float max_speed, float max_turn) {
	
	this->max_speed = max_speed;
	this->max_turn = max_turn;
	this->bound_right = 
		(this->max_speed + this->max_turn * this->wheel_distance/2.0) / this->wheel_radius;
	this->bound_left = 
		(this->max_speed - this->max_turn * this->wheel_distance/2.0) / this->wheel_radius;
}