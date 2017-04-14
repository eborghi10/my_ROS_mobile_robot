#pragma once

#include <AS5048A.h>
#include <ros.h>
#include <std_msgs/Float32.h>

#define M_PI 3.14159265359

// 0 to +2*Pi
const boolean ZERO_TO_2PI PROGMEM = false;
// -Pi to +Pi
const boolean PLUS_MINUS_PI PROGMEM = true;

class MagneticEncoder
{
	float read2angle(uint16_t);
	float normalize(float);

	boolean angleMode;
	float initial_angle;
	AS5048A *Encoder;

public:
	MagneticEncoder();
	MagneticEncoder(uint8_t);
	MagneticEncoder(uint8_t, boolean);

	float GetAngle();
};

///////////////////////////////////////////////////////////////

MagneticEncoder::MagneticEncoder()
	: MagneticEncoder::MagneticEncoder(CS1, PLUS_MINUS_PI) {}

MagneticEncoder::MagneticEncoder(uint8_t digitalPin)
	: MagneticEncoder::MagneticEncoder(digitalPin, PLUS_MINUS_PI) {}

MagneticEncoder::MagneticEncoder(uint8_t digitalPin, boolean mode)
	: Encoder(new AS5048A(digitalPin)), angleMode(mode) {

	Encoder->init();

	initial_angle = 
		MagneticEncoder::read2angle( Encoder->getRawRotation() );
}

float MagneticEncoder::GetAngle() {

	float current_angle = 
		MagneticEncoder::read2angle( Encoder->getRawRotation() );

	return normalize(current_angle - initial_angle);
}

float MagneticEncoder::read2angle(uint16_t register_output) {

	return register_output * ((float)2*M_PI / 16383);
}

float MagneticEncoder::normalize(float angle) {

	if (angleMode == PLUS_MINUS_PI)	angle += M_PI;

	angle = fmod(angle, 2*PI);
	
	if (angle < 0) angle += 2*PI;

	if (angleMode == PLUS_MINUS_PI)	angle -= M_PI;

	return angle;
}