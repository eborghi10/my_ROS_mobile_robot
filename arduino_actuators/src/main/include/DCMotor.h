#pragma once

#include "MagneticEncoder.h"

//////////////////////////////////////////////////////////////////

//#define _16BIT_PWM_

#ifdef _16BIT_PWM_
  #define MAX_VALUE 0xFF
#else
  #define MAX_VALUE 0xFFFF
#endif

//////////////////////////////////////////////////////////////////

class DCMotor {

  MagneticEncoder *encoder;

  int INL; 
  int INH;
  void initPins();

public:

  DCMotor();
  DCMotor(int,int);
  DCMotor(int,int,MagneticEncoder*);
  DCMotor(MagneticEncoder*);
  void CW();
  void CCW();
  void Stop();
  void PWM(uint8_t);

  char* GetEncoderTopicName();
  double GetEncoderAngle();
};

//////////////////////////////////////////////////////////////////

DCMotor::DCMotor()
  : DCMotor::DCMotor((MagneticEncoder *)NULL) {}

DCMotor::DCMotor(int INL, int INH) 
  : DCMotor::DCMotor(INL, INH, NULL) {}

DCMotor::DCMotor(MagneticEncoder* encoder) 
  : DCMotor::DCMotor(13, 12, encoder) {}

DCMotor::DCMotor(int INL, int INH, MagneticEncoder* encoder)
  : INL(INL), 
    INH(INH),
    encoder(encoder) {

      DCMotor::initPins();
    }

//////////////////////////////////////////////////////////////////

void DCMotor::initPins() {

  pinMode(this->INH, OUTPUT);
  pinMode(this->INL, OUTPUT);
  DCMotor::Stop();
}

void DCMotor::CW() {
  // Motor gira en un sentido
  DCMotor::PWM(MAX_VALUE);
}

void DCMotor::Stop() {
  // Motor no gira
  DCMotor::PWM(0);
}

void DCMotor::CCW() {
  // Motor gira en sentido inverso
  digitalWrite (INL, HIGH);
  digitalWrite (INH, LOW);
}

void DCMotor::PWM(uint8_t val) {
	// For security reasons
	val > MAX_VALUE? val = MAX_VALUE : val;
	
	digitalWrite(INL, LOW);
	analogWrite(INH, val);
}

char* DCMotor::GetEncoderTopicName() {

  return encoder->GetTopicName();
}

double DCMotor::GetEncoderAngle() {

  return encoder->GetAngle();
}
