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

ros::NodeHandle nh;
std_msgs::Float32 msg_left;
std_msgs::Float32 msg_right;
ros::Publisher pub_left("/encoder/left", &msg_left);
ros::Publisher pub_right("/encoder_right", &msg_right);

MagneticEncoder *encoder_left = 
	new MagneticEncoder(
		CS1, 
		LEFT,
		ZERO_TO_2PI
	);

MagneticEncoder *encoder_right = 
	new MagneticEncoder(
		CS2, 
		RIGHT,
		ZERO_TO_2PI
	);

void setup()
{	
	nh.initNode();
  	nh.advertise(pub_left);
  	nh.advertise(pub_right);
}

void loop()
{
	nh.spinOnce();

	msg_left.data = encoder_left->GetAngle();
	pub_left.publish(&msg_left);

	msg_right.data  = encoder_right->GetAngle();
	pub_right.publish(&msg_right);
}
