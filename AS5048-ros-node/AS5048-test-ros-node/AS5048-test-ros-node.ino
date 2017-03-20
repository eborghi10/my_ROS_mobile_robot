#include <AS5048A.h>
#include <ros.h>
#include <std_msgs/String.h>
//#include <std_msgs::UInt16.h>

/**
 * ROS node to test the magnetic encoder AS5048A:
 * - publish angle values to a new topic
 *
 * Improvement (2 Encoders):
 * https://github.com/ZoetropeLabs/AS5048A-Arduino/issues/1
 *
 */

AS5048A angleSensor(10);
uint16_t zero_position;

ros::NodeHandle nh;
std_msgs::String msg;
ros::Publisher pub("/angle_values", &msg);


void setup()
{
	angleSensor.init();
  	zero_position = angleSensor.getZeroPosition();

  	nh.initNode();
  	nh.advertise(pub);
}


void loop()
{
	nh.spinOnce();

//	delay(1000);

	/**
	 * Returns the value between 0-360 degrees.
	 *
	 */
	uint16_t val1 = angleSensor.getRawRotation();
	msg.data = "> Raw: ";
	itoa(msg.data, val1 - zero_position, sizeof(uint16_t));
	pub.publish(&msg);
	nh.loginfo(msg.data);

	/**
	 * Returns a relative angle to 0 (signed int, +/-180 degrees)
	 * which is useful if you're making a PID system.
	 *
	 */
	int16_t val2 = angleSensor.getRotation();
	msg.data = "> Int: ";
	itoa(msg.data, val2 - zero_position, sizeof(uint16_t));
	pub.publish(&msg);
}
