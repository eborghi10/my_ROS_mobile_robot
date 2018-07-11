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
#include "include/DCMotorWithEncoder.h"
#include "include/MagneticEncoder.h"

DCMotorWithEncoder *motor_left;
DCMotorWithEncoder *motor_right;

void setup()
{
    nh.initNode();

    motor_left = new DCMotorWithEncoder(IN1, IN2, ENA, 
                 new MagneticEncoder(CS1, LEFT), 
                 LEFT);

    motor_right = new DCMotorWithEncoder(IN3, IN4, ENB, 
                  new MagneticEncoder(CS2, RIGHT), 
                  RIGHT);

    last_time = millis();
}

void loop() 
{
    const double current_time = millis() / 1E3;
    const double delta_time = (current_time - last_time);
    if(delta_time >= 1.0/rate)
    {
        // Publish angles
        motor_left->PublishAngle();
        motor_right->PublishAngle();
        // Update last time
        last_time = current_time;
    }
    nh.spinOnce();
}
