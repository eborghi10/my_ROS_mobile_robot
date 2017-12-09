#include "MagneticEncoder.h"

/**
 * ROS Base Units (REP 103):
 *
 * - angle: rad
 * - ang. velocity: rad/s
 *
 * http://wiki.ros.org/urdf/Tutorials/Using%20urdf%20with%20robot_state_publisher
 *
 */

MagneticEncoder *encoder_left;
MagneticEncoder *encoder_right;

void setup()
{	
	encoder_left = new MagneticEncoder(
		CS1, 
		LEFT,
		ZERO_TO_2PI,
		(char*)"/encoder/left"
	);

	encoder_right = new MagneticEncoder(
		CS2, 
		RIGHT,
		ZERO_TO_2PI,
		(char*)"/encoder/right"
	);
}

void loop()
{
	(encoder_left->nh).spinOnce();
	(encoder_right->nh).spinOnce();

	encoder_left->PublishAngle();
	encoder_right->PublishAngle();
}
