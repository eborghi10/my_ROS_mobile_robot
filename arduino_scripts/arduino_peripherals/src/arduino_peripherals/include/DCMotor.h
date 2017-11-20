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
  void CW(INT_PWM);
  void CCW(INT_PWM);
  INT_PWM protectOutput(INT_PWM);

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

void DCMotor::CW(INT_PWM val) {
  // Motor turns forward or CW
  analogWrite(this->INL, LOW);
  analogWrite(this->INH, protectOutput(val));
}

void DCMotor::CCW(INT_PWM val) {
  // Motor turns in the inverse direction or CCW
  analogWrite (this->INL, protectOutput(val));
  analogWrite (this->INH, LOW);
}

INT_PWM DCMotor::protectOutput(INT_PWM val) {

  // For security reasons
  val > MAX_VALUE? val = MAX_VALUE : val;

  return val;
}

void DCMotor::PublishAngle() {

  this->encoder->PublishAngle();
}

void DCMotor::motorCb(const std_msgs::Float32& msg) {

//  INT_PWM right_map = 
//    map(static_cast<INT_PWM>(u_r), 0, boundRight, 0, MAX_VALUE);
//  INT_PWM left_map = 
//    map(static_cast<INT_PWM>(u_l), 0, boundLeft, 0, MAX_VALUE);

  if (strcmp(this->name, "left") == 0) 
  {  
    msg.data? DCMotor::CW(msg.data) : DCMotor::CCW(msg.data);
  } else 
  {
    msg.data? DCMotor::CCW(msg.data) : DCMotor::CW(msg.data);
  }
}