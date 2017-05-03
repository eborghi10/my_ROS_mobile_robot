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
 * 1) Compile project in the ROS workspace (catkin_make)
 * 2) Upload to Arduino
 * 3) $ roslaunch arduino_actuators execute.launch
 */

#include "include/DCMotor.h"

//////////////////////////////////////////////////////////////////

DCMotor *motor_left;
//DCMotor *motor_right;

//////////////////////////////////////////////////////////////////

void setup()
{
    nh.initNode();

    motor_left = new DCMotor(IN1, IN2, 
                 new MagneticEncoder(CS1),
                 "left");
    /*
    motor_right = new DCMotor(IN3, IN4,
                  new MagneticEncoder(CS2),
                  "right");
    */
}

//////////////////////////////////////////////////////////////////

void loop() 
{
	nh.spinOnce();

  //motor_left->PublishAngle();
  //motor_right->PublishAngle();

  delay(50);
}