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
sensor_msgs::JointState msg;

MagneticEncoder encoder_left(
	10, 
	LEFT,
	ZERO_TO_2PI
);

MagneticEncoder encoder_right(
	11, 
	RIGHT,
	ZERO_TO_2PI
);

ros::Publisher pub("/encoder", &msg);

void setup()
{	
	nh.initNode();

  	nh.advertise(pub);
}


ros::Time prevTime(0,0);
double dT;
double prevPosition[] = {0, 0};

void loop()
{
	/**
	 * TODO: INSERT THE FOLLOWING CODE INTO A FUNCTION()
	 *	
	 */

	nh.spinOnce();

	msg.header.stamp 	= nh.now();

	dT = msg.header.stamp.toSec() - prevTime.toSec();

	msg.name[LEFT] 		= encoder_left.topic_name;
	msg.position[LEFT]  = encoder_left.GetAngle();
	msg.velocity[LEFT]  
		= (msg.position[LEFT] - prevPosition[LEFT]) / dT;

	msg.name[RIGHT] 	 = encoder_right.topic_name;
	msg.position[RIGHT]  = encoder_right.GetAngle();
	msg.velocity[RIGHT]  
		= (msg.position[RIGHT] - prevPosition[RIGHT]) / dT;

	prevPosition[LEFT] 	= msg.position[LEFT];
	prevPosition[RIGHT] = msg.position[RIGHT];

	prevTime = msg.header.stamp;

	pub.publish(&msg);
}
