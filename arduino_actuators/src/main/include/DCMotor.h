#pragma once

#include "ATmega2560-HW.h"
#include "MagneticEncoder.h"

//////////////////////////////////////////////////////////////////

//#define _16BIT_PWM_

#ifdef _16BIT_PWM_
  #define MAX_VALUE 0xFFFF
  #define INT_PWM   uint16_t
#else
  #define MAX_VALUE 0xFF
  #define INT_PWM   uint8_t
#endif

//////////////////////////////////////////////////////////////////

class DCMotor {

  MagneticEncoder *encoder;

  int INL; 
  int INH;

  void initPins();
  INT_PWM protectOutput(INT_PWM);

public:

  DCMotor();
  DCMotor(int,int);
  DCMotor(MagneticEncoder*);
  DCMotor(int,int,MagneticEncoder*);

  void Stop();
  void CW(INT_PWM);
  void CCW(INT_PWM);

  float GetEncoderAngle();
};

//////////////////////////////////////////////////////////////////

DCMotor::DCMotor()
  : DCMotor::DCMotor((MagneticEncoder *)NULL) {}

DCMotor::DCMotor(int INL, int INH) 
  : DCMotor::DCMotor(INL, INH, (MagneticEncoder*)NULL) {}

DCMotor::DCMotor(MagneticEncoder* encoder) 
  : DCMotor::DCMotor(IN1, IN2, encoder) {}

DCMotor::DCMotor(int INL, int INH, MagneticEncoder* encoder)
  : INL(INL), INH(INH), encoder(encoder) {

      DCMotor::initPins();
    }

//////////////////////////////////////////////////////////////////

void DCMotor::initPins() {

  pinMode(this->INH, OUTPUT);
  pinMode(this->INL, OUTPUT);
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

float DCMotor::GetEncoderAngle() {

  return encoder->GetAngle();
}
