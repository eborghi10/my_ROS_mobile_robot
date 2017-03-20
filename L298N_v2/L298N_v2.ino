/**
 * To execute node in ROS:
 * 
 * $ roscore  (in a terminal)
 * $ rosrun roserial_python serial_node.py /dev/ttyACM0 (in another terminal)
 * 
 * ----------------------------------------------
 * 
 * The program waits for data via "/dc_motor" topic, eg:
 * $ rostopic pub "dc_motor" std_msgs/UInt16 "data:255"
 * 
 * And sends it via PWM signal, so, it defines the velocity of the motor.
 * 
 * TODO: make a roslaunch executable.
 * 
 */

#include "DCMotor.h"  // DC motor class
#include <ros.h>
#include <std_msgs/UInt16.h>

void motor_callback(const std_msgs::UInt16& msg);

ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt16> sub("dc_motor", motor_callback);
DCMotor *motor1;

void setup()
{
  motor1 = new DCMotor(5,4);
// ANother way of creating a DC motor
//  DCMotor motor2(6,7);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() 
{  
  nh.spinOnce();
  delay(1);
}

void motor_callback(const std_msgs::UInt16& msg) {
  motor1->PWM(msg.data);
}

