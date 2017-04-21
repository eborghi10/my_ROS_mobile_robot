#pragma once

#include "MagneticEncoder.h"

//////////////////////////////////////////////////////////////////

//#define _16BIT_PWM_

#ifdef _16BIT_PWM_
  #define INT_PWM   uint16_t
  const INT_PWM MAX_VALUE PROGMEM = 0xFFFF;
#else
  #define INT_PWM   uint8_t
  const INT_PWM MAX_VALUE PROGMEM = 0xFF;
#endif

//////////////////////////////////////////////////////////////////

class DCMotor {

  MagneticEncoder *encoder;

  int INL; 
  int INH;
  char* name;

  void initPins();
  void Stop();
  void CW(INT_PWM);
  void CCW(INT_PWM);
  INT_PWM protectOutput(INT_PWM);

  void motorCb(const robot_msgs::Arduino&);

  ros::Subscriber<robot_msgs::Arduino, DCMotor> sub;

public:

  DCMotor();
  DCMotor(int,int);
  DCMotor(MagneticEncoder*);
  DCMotor(int,int,MagneticEncoder*);
  DCMotor(int,int,MagneticEncoder*,char*);

  void PublishAngle();
};

//////////////////////////////////////////////////////////////////

DCMotor::DCMotor()
  : DCMotor::DCMotor(new MagneticEncoder(CS1)) {}

DCMotor::DCMotor(int INL, int INH) 
  : DCMotor::DCMotor(INL, INH, (MagneticEncoder*)NULL) {}

DCMotor::DCMotor(MagneticEncoder* encoder) 
  : DCMotor::DCMotor(IN1, IN2, encoder) {}

DCMotor::DCMotor(int INL, int INH, MagneticEncoder* encoder)
  : DCMotor::DCMotor(INL, INH, encoder, "left") {}
    

DCMotor::DCMotor(int INL, int INH, MagneticEncoder* encoder, char* name)
  : INL(INL), INH(INH), encoder(encoder), name(name),
    sub("/cmd_vel_mux/input/teleop", &DCMotor::motorCb, this) 
    {
      DCMotor::initPins();
      nh.subscribe(sub);
    }

//////////////////////////////////////////////////////////////////

void DCMotor::initPins() {

  pinMode(this->INH, OUTPUT);
  pinMode(this->INL, OUTPUT);
  DCMotor::Stop();
}

void DCMotor::Stop() {
  // Motor don't stop
  analogWrite (INL, LOW); 
  analogWrite (INH, LOW);
}

void DCMotor::CW(INT_PWM val) {
  // Motor turns forward or CW
  analogWrite(INL, LOW);
  analogWrite(INH, protectOutput(val));
}

void DCMotor::CCW(INT_PWM val) {
  // Motor turns in the inverse direction or CCW
  analogWrite (INL, protectOutput(val));
  analogWrite (INH, LOW);
}

INT_PWM DCMotor::protectOutput(INT_PWM val) {

  // For security reasons
  val > MAX_VALUE? val = MAX_VALUE : val;

  return val;
}

void DCMotor::PublishAngle() {

  encoder->PublishAngle(name);
}

//////////////////////////////////////////////////////////////////

void DCMotor::motorCb(const robot_msgs::Arduino& msg) {

  if (strcmp(msg.name, "left") == 0) {
    
    msg.data? DCMotor::CW(msg.data) : DCMotor::CCW(msg.data);
  
  } else {

    msg.data? DCMotor::CCW(msg.data) : DCMotor::CW(msg.data);
  }
}