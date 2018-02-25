#pragma once

#include <AS5048A.h>
#include <std_msgs/Float32.h>

const boolean LEFT PROGMEM = false;
const boolean RIGHT PROGMEM = true;

const boolean ZERO_TO_2PI PROGMEM = false;      // 0 to +2*Pi
const boolean PLUS_MINUS_PI	PROGMEM = true;     // -Pi to +Pi

class MagneticEncoder
{
	float read2angle(uint16_t);
	float normalize(float);
    float GetAngle();

	boolean angleMode;
	boolean position;
	float initial_angle;
	
    AS5048A *Encoder;
    std_msgs::Float32 msg;
    ros::Publisher pub;

public:
	MagneticEncoder(uint8_t, boolean);
	MagneticEncoder(uint8_t, boolean, boolean);

    void PublishAngle();
};

MagneticEncoder::MagneticEncoder(
	uint8_t digitalPin, boolean position)
	: MagneticEncoder::MagneticEncoder(digitalPin, position, PLUS_MINUS_PI) {}

MagneticEncoder::MagneticEncoder(
	uint8_t digitalPin, boolean position, boolean mode)
	: Encoder(new AS5048A(digitalPin))
    , angleMode(mode) 
    , pub(position == LEFT ? "my_robot/left_wheel_angle" 
                    : "my_robot/right_wheel_angle", &msg)    
{
    Encoder->init();
    this->initial_angle = MagneticEncoder::read2angle( Encoder->getRawRotation() );
    nh.advertise(pub);
}

float MagneticEncoder::GetAngle() {
	// Change sign of the sensor reading
	int8_t k;
	this->position == LEFT? k = -1 : k = 1;

	float current_angle = k * MagneticEncoder::read2angle( Encoder->getRawRotation() );

	return normalize(current_angle - this->initial_angle);
}

float MagneticEncoder::read2angle(uint16_t register_output) {
    // M_PI defined in math.h for newer versions
	return register_output * ((float)2*M_PI / 16383);
}

float MagneticEncoder::normalize(float angle) {

	if (this->angleMode == PLUS_MINUS_PI) angle += M_PI;
	angle = fmod(angle, 2*PI);
	if (angle < 0) angle += 2*PI;
	if (this->angleMode == PLUS_MINUS_PI) angle -= M_PI;

	return angle;
}

void MagneticEncoder::PublishAngle() {
    msg.data = GetAngle();
    pub.publish(&msg);
}
