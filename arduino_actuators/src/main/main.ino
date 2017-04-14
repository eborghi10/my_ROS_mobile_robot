/**
 * http://answers.ros.org/question/232045/rosserial-multiple-publisher-and-subscriber/
 *
 * > Upload to Arduino and then:
 *
 * $ roscore
 * $ rosrun rosserial_python serial_node.py /dev/ttyACM0
 *
 * ---------------------------------------------------------------
 *
 * Or BETTER:
 *
 * 1) Upload to Arduino
 * 2) Compile project in the ROS workspace (catkin_make)
 * 3) $ roslaunch arduino_actuators execute.launch
 */

#include "include/DifferentialDriveRobot.h"

//////////////////////////////////////////////////////////////////

DifferentialDriveRobot *my_robot;

//////////////////////////////////////////////////////////////////

void setup()
{
  my_robot = new DifferentialDriveRobot(
    new DCMotor(IN1, IN2, new MagneticEncoder(CS1)),
    new DCMotor(IN3, IN4, new MagneticEncoder(CS2))
  );

  my_robot->UpdatePhysicalParameters(0.2, 1.0);

  nh.initNode();

  nh.subscribe(sub);
  nh.advertise(pub_left);
  nh.advertise(pub_right);
}

//////////////////////////////////////////////////////////////////

void loop() 
{
	nh.logdebug("Initializing program...");
	nh.spinOnce();

  my_robot->SendAngles();

  if( !nh.connected() )  my_robot->Stop();
}

//////////////////////////////////////////////////////////////////

void ddr_callback(const geometry_msgs::Twist& msg_motor) {

  my_robot->Move(msg_motor.linear.x, msg_motor.angular.z);
}