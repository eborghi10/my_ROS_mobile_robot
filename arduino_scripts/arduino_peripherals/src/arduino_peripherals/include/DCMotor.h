#pragma once

#include "MagneticEncoder.h"

class DCMotor {

  MagneticEncoder *encoder;
  ros::Subscriber<std_msgs::Float32, DCMotor> sub;

  int INL; 
  int INH;
  char* name;

  void initPins();
  void Stop();
  void CW(float);
  void CCW(float);
  int mapToInt(float);

  void motorCb(const std_msgs::Float32&);

public:

  DCMotor();
  DCMotor(int,int);
  DCMotor(MagneticEncoder*);
  DCMotor(int,int,MagneticEncoder*);
  DCMotor(int,int,MagneticEncoder*,char*);

  void PublishAngle();
};

DCMotor::DCMotor(int INL, int INH, MagneticEncoder* encoder, char* name)
  : INL(INL)
  , INH(INH)
  , encoder(encoder)
  , name(name)
  , sub(strcmp(name, "left") == 0 ? "my_robot/left_wheel_vel" 
                : "my_robot/right_wheel_vel", &DCMotor::motorCb, this)
{
    DCMotor::initPins();
    nh.subscribe(sub);
}

void DCMotor::initPins() {

  pinMode(this->INH, OUTPUT);
  pinMode(this->INL, OUTPUT);
  DCMotor::Stop();
}

void DCMotor::Stop() {
  // Motor don't stop
  analogWrite (this->INL, LOW); 
  analogWrite (this->INH, LOW);
}

void DCMotor::CW(float val) {
  // Motor turns forward or CW
  analogWrite(this->INL, LOW);
  analogWrite(this->INH, DCMotor::mapToInt(val));
}

void DCMotor::CCW(float val) {
  // Motor turns in the inverse direction or CCW
  analogWrite (this->INL, DCMotor::mapToInt(val));
  analogWrite (this->INH, LOW);
}

void DCMotor::PublishAngle() {

  this->encoder->PublishAngle();
}

void DCMotor::motorCb(const std_msgs::Float32& msg) {
  if (strcmp(this->name, "left") == 0) 
  {  
    msg.data? DCMotor::CCW(msg.data) : DCMotor::CW(msg.data);
  } else 
  {
    msg.data? DCMotor::CW(msg.data) : DCMotor::CCW(msg.data);
  }
}

/**
 * Maps the positive velocity command from
 * [0, 1.0] to [0, 255].
 *
 * @param val The velocity command
 * @return The mapped velocity command
 */
int DCMotor::mapToInt(float val) {
  return map(static_cast<int>(val), 0, 1, 0, 255);
}
