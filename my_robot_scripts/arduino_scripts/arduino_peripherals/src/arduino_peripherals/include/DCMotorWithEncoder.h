#pragma once

#include "MagneticEncoder.h"
#include <DCMotor.h>
#include <std_msgs/Int16.h>

class DCMotorWithEncoder {
  // DC Motor
  DCMotor *motor;
  // Encoder device
  MagneticEncoder *encoder;
  // ROS subscriber
  ros::Subscriber<std_msgs::Int16, DCMotorWithEncoder> sub;
  // Callback
  void motorCb(const std_msgs::Int16&);
public:
  // Constructor
  DCMotorWithEncoder(int,int,int,MagneticEncoder*,bool);
  // Public method
  void PublishAngle();
};

DCMotorWithEncoder::DCMotorWithEncoder(int INL, int INH, int EN, MagneticEncoder* encoder, bool position)
  : encoder(encoder)
  , motor(EN, INL, INH)
  , sub(position == LEFT ? "my_robot/left_wheel_vel" : "my_robot/right_wheel_vel", &DCMotorWithEncoder::motorCb, this)
{
    nh.subscribe(sub);
    // Right motor turns to the other side
    if (position == RIGHT) this->motor->setClockwise(false);
}

void DCMotorWithEncoder::PublishAngle() {
  // Publish encoder angle
  this->encoder->PublishAngle();
}

void DCMotorWithEncoder::motorCb(const std_msgs::Int16& msg) {
  // Send command to the motor
  this->motor->setSpeed(msg.data);
}
