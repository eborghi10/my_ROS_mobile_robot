#pragma once

#include <AS5048A.h>
#include <spline.h>
#include <std_msgs/Float32.h>

class MagneticEncoder
{
    double GetAngle();
    // LEFT or RIGHT
	boolean position;
	// Encoder device AS5048A
    AS5048A *Encoder;
    // Spline for linearizing the encoder output
    Spline<double> spline;
    // ROS message
    std_msgs::Float32 msg;
    ros::Publisher pub;
public:
	// Constructor
    MagneticEncoder(uint8_t, boolean);
    // Public method
    void PublishAngle();
};

MagneticEncoder::MagneticEncoder(uint8_t digitalPin, boolean position)
	: Encoder(new AS5048A(digitalPin))
    , spline(position == LEFT ? REAL_LEFT : REAL_RIGHT, 
             position == LEFT ? IDEAL_LEFT : IDEAL_RIGHT,
             (position == LEFT ? sizeof(IDEAL_LEFT) : sizeof(IDEAL_RIGHT))/sizeof(double))
    , pub(position == LEFT ? "my_robot/left_wheel_angle" : "my_robot/right_wheel_angle", &msg)    
{
    Encoder->begin();
    nh.advertise(pub);
}

double MagneticEncoder::GetAngle() {
    // Read device output
	const double current_angle = Encoder->getRotationInRadians();
    // Normalize encoder output (Change sign of the sensor reading)
	return (this->position == LEFT? 1 : -1) * spline.value(current_angle);
}

void MagneticEncoder::PublishAngle() {
    msg.data = GetAngle();
    pub.publish(&msg);
}
