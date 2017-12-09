#pragma once

#include <AS5048A.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include "ATmega2560-HW.h"

// TODO: Use PROGMEM for this #define

#define LEFT 	(uint8_t)0
#define RIGHT 	(uint8_t)1
#define NONE	(uint8_t)2

#define ZERO_TO_2PI		(uint8_t)0	// 0 to +2*Pi
#define PLUS_MINUS_PI	(uint8_t)1	// -Pi to +Pi

class MagneticEncoder
{
	float read2angle(uint16_t);
	float normalize(float);

	uint8_t angleMode;
	uint8_t position;
	float initial_angle;
	AS5048A *Encoder;

public:
	MagneticEncoder();
	MagneticEncoder(uint8_t, uint8_t);
	MagneticEncoder(uint8_t, uint8_t, uint8_t);
	MagneticEncoder(uint8_t, uint8_t, uint8_t, char*);

	float GetAngle();
	void PublishAngle();

	ros::NodeHandle nh;
	std_msgs::Float32 msg;
	ros::Publisher pub;
};

///////////////////////////////////////////////////////////////

MagneticEncoder::MagneticEncoder()
	: MagneticEncoder::MagneticEncoder(CS1, NONE, PLUS_MINUS_PI) {}

MagneticEncoder::MagneticEncoder(
	uint8_t digitalPin, uint8_t position)
	: MagneticEncoder::MagneticEncoder(digitalPin, position, PLUS_MINUS_PI) {}

MagneticEncoder::MagneticEncoder(
	uint8_t digitalPin, uint8_t position, uint8_t mode)
	: MagneticEncoder::MagneticEncoder(digitalPin, position, mode, (char*)"/encoder") {}

MagneticEncoder::MagneticEncoder(
	uint8_t digitalPin, uint8_t position, uint8_t mode, char* topic)
	: Encoder(new AS5048A(digitalPin)), angleMode(mode),
	  pub(topic, &msg) {

	Encoder->init();

	initial_angle = 
		MagneticEncoder::read2angle( Encoder->getRawRotation() );

	nh.initNode();
  	nh.advertise(pub);
}

///////////////////////////////////////////////////////////////

float MagneticEncoder::GetAngle() {

	float current_angle = 
		MagneticEncoder::read2angle( Encoder->getRawRotation() );

	return normalize(current_angle - initial_angle);
}

float MagneticEncoder::read2angle(uint16_t register_output) {

	return register_output * ((float)2*M_PI / 16383);
}

float MagneticEncoder::normalize(float angle) 
{

	if (angleMode == PLUS_MINUS_PI)	angle += M_PI;

	angle = fmod(angle, 2*PI);
	
	if (angle < 0) angle += 2*PI;

	if (angleMode == PLUS_MINUS_PI)	angle -= M_PI;

	return angle;
}

void MagneticEncoder::PublishAngle(void) {

	msg.data  = MagneticEncoder::GetAngle();
	pub.publish(&msg);
}
