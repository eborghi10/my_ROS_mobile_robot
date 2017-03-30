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
    new DCMotor(4,5, new MagneticEncoder(10, LEFT, ZERO_TO_2PI), LEFT),
    new DCMotor(6,7, new MagneticEncoder(11, RIGHT, ZERO_TO_2PI), RIGHT)
  );

  my_robot->UpdatePhysicalParameters(0.2, 1.0);
}

//////////////////////////////////////////////////////////////////

void loop() 
{
	(my_robot->nh).loginfo("Initializing program...");
	(my_robot->nh).spinOnce();

  	my_robot->SendAngles();

  	if( !(my_robot->nh).connected() )  my_robot->Stop();
}

//////////////////////////////////////////////////////////////////

// TODO: IT'S NECESSARY TO DECLARE THE CALLBACK IN THE MAIN LOOP?
//		 TRY TO PUT INTO DifferentialDriveRobot.h

void ddr_callback(const geometry_msgs::Twist& msg_motor) {

	// TODO: WHAT IS THE PURPOSE OF var??? For testing...

  int var = my_robot->Move(msg_motor.linear.x, msg_motor.angular.z);
}