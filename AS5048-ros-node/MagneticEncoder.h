#pragma once

#include <AS5048A.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>

#define LEFT 	(uint8_t)0
#define RIGHT 	(uint8_t)1
#define NONE	(uint8_t)2

#define ZERO_TO_2PI		(uint8_t)0	// 0 to +2*Pi
#define PLUS_MINUS_PI	(uint8_t)1	// -Pi to +Pi

#define M_PI 	3.14159265359


class MagneticEncoder
{
	double read2angle(uint16_t);
	double normalize(double);

	uint8_t angleMode;
	uint8_t position;
	uint16_t initial_angle;
	AS5048A *Encoder;

public:
	MagneticEncoder();
	MagneticEncoder(uint8_t, uint8_t);
	MagneticEncoder(uint8_t, uint8_t, uint8_t);

	double GetAngle();

	char* topic_name;
};

///////////////////////////////////////////////////////////////

MagneticEncoder::MagneticEncoder()
	: MagneticEncoder(10, NONE, PLUS_MINUS_PI) {}

MagneticEncoder::MagneticEncoder(
	uint8_t digitalPin, uint8_t position)
	: MagneticEncoder(digitalPin, position, PLUS_MINUS_PI) {}

MagneticEncoder::MagneticEncoder(
	uint8_t digitalPin, uint8_t position, uint8_t mode)
	: Encoder(new AS5048A(digitalPin)), angleMode(mode) {

	Encoder->init();

	if (position == LEFT)		char *topic_name = "encoder_left";
	else if(position == RIGHT)	char *topic_name = "encoder_right";
	else						char *topic_name = "encoder";

	initial_angle = 
		MagneticEncoder::read2angle( Encoder->getRawRotation() );
}

double MagneticEncoder::GetAngle() {

	double current_angle = 
		MagneticEncoder::read2angle( Encoder->getRawRotation() );

	return normalize(current_angle - initial_angle);
}

double MagneticEncoder::read2angle(uint16_t angle) {

	return angle * ((double)2*M_PI / 16383);
}

double MagneticEncoder::normalize(double angle) 
{

	if (angleMode == PLUS_MINUS_PI)	angle += M_PI;

	angle = fmod(angle, 2*PI);
	
	if (angle < 0) angle += 2*PI;

	if (angleMode == PLUS_MINUS_PI)	angle -= M_PI;

	return angle;
}