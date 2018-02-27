#pragma once

#include <AS5048A.h>
#include <ros.h>
#include <std_msgs/Float32.h>

const bool LEFT PROGMEM = false;
const bool RIGHT PROGMEM = true;

class MagneticEncoder
{
private:
	float ticks2angle(uint16_t);
	float normalize(float);

	bool position;
	float initial_angle;
	AS5048A *Encoder;

	ros::Publisher pub;
	std_msgs::Float32 msg;

public:
	MagneticEncoder(uint8_t, bool);

	float GetAngle();
	void PublishAngle();

	ros::NodeHandle nh;
};

///////////////////////////////////////////////////////////////

MagneticEncoder::MagneticEncoder(uint8_t digitalPin, bool position)
	: Encoder(new AS5048A(digitalPin))
	, position(position)
	, pub(position == LEFT ? "/encoder/left" : "/encoder/right", &msg) 
{
	Encoder->init();

	initial_angle = 
		MagneticEncoder::ticks2angle(Encoder->getRawRotation());

	nh.initNode();
  	nh.advertise(pub);
}

///////////////////////////////////////////////////////////////

float MagneticEncoder::GetAngle() {

	float current_angle = 
		MagneticEncoder::ticks2angle(Encoder->getRawRotation());

	return normalize(current_angle - initial_angle);
}

float MagneticEncoder::ticks2angle(uint16_t register_output) {

	return register_output * ((float)2*M_PI / 16383);
}

float MagneticEncoder::normalize(float angle) 
{
	angle = fmod(angle + M_PI, 2*PI);	
	if (angle < 0) angle += 2*PI;
	return angle - M_PI;
}

void MagneticEncoder::PublishAngle(void) 
{
	msg.data  = MagneticEncoder::GetAngle();
	pub.publish(&msg);
}
