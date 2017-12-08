/*
 * 1) Upload the code to Arduino
 * 2) Compile project in the ROS workspace (catkin_make)
 * 3) $ roslaunch arduino_peripherals run_arduino_node.launch
 * 
 * Expected output:
 * 
 * // TODO: Fill with rosnode info ...
 */

// Include libraries
#include "include/ArduinoHW.h"
#include "include/DCMotor.h"
#include "include/MagneticEncoder.h"

DCMotor *motor_left;
DCMotor *motor_right;

void setup()
{
    nh.initNode();

    motor_left = new DCMotor(IN1, IN2, 
                 new MagneticEncoder(CS1, LEFT),
                 (char*)"left");

    motor_right = new DCMotor(IN3, IN4,
                  new MagneticEncoder(CS2, RIGHT),
                  (char*)"right");
}

void loop() 
{
    nh.spinOnce();

    motor_left->PublishAngle();
    motor_right->PublishAngle();

    delay(50);
}
