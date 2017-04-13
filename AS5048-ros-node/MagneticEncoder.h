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

#define M_PI 	3.14159265359

/**
 * TODO: "position" is not used
 *
 */

class MagneticEncoder
{
	float read2angle(uint16_t);
	float normalize(float);

	uint8_t angleMode;
	uint8_t position;
	uint16_t initial_angle;
	AS5048A *Encoder;

public:
	MagneticEncoder();
	MagneticEncoder(uint8_t, uint8_t);
	MagneticEncoder(uint8_t, uint8_t, uint8_t);

	float GetAngle();
};

///////////////////////////////////////////////////////////////

MagneticEncoder::MagneticEncoder()
	: MagneticEncoder::MagneticEncoder(CS1, NONE, PLUS_MINUS_PI) {}

MagneticEncoder::MagneticEncoder(
	uint8_t digitalPin, uint8_t position)
	: MagneticEncoder::MagneticEncoder(digitalPin, position, PLUS_MINUS_PI) {}

MagneticEncoder::MagneticEncoder(
	uint8_t digitalPin, uint8_t position, uint8_t mode)
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

float MagneticEncoder::read2angle(uint16_t angle) {

	return angle * ((float)2*M_PI / 16383);
}

float MagneticEncoder::normalize(float angle) 
{

	if (angleMode == PLUS_MINUS_PI)	angle += M_PI;

	angle = fmod(angle, 2*PI);
	
	if (angle < 0) angle += 2*PI;

	if (angleMode == PLUS_MINUS_PI)	angle -= M_PI;

	return angle;
}