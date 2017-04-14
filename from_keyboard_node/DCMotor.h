#pragma once

#include "ATmega2560-HW.h"

//#define _16BIT_PWM_

#ifdef _16BIT_PWM_
  #define MAX_VALUE 0xFFFF
  #define INT_PWM   uint16_t
#else
  #define MAX_VALUE 0xFF
  #define INT_PWM   uint8_t
#endif

class DCMotor {
public:
  DCMotor();
  DCMotor(int,int);
  void CW(INT_PWM);
  void CCW(INT_PWM);
  void Stop();
private:  
  int INL; 
  int INH;
  void initPins();
  INT_PWM protectOutput(INT_PWM);
};

DCMotor::DCMotor()
  : DCMotor::DCMotor(IN1, IN2) {
  }

DCMotor::DCMotor(int INL, int INH) {
  this->INL = INL;
  this->INH = INH;
  DCMotor::initPins();
}

void DCMotor::initPins() {

  pinMode(this->INL, OUTPUT);
  pinMode(this->INH, OUTPUT);
  DCMotor::Stop();
}

void DCMotor::Stop() {
  // Motor no gira
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