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
  uint8_t direction;
  void initPins();

public:

  DCMotor();
  DCMotor(uint8_t);
  DCMotor(int,int);
  DCMotor(int,int, uint8_t);
  DCMotor(int,int,MagneticEncoder*);
  DCMotor(int,int,MagneticEncoder*, uint8_t);
  DCMotor(MagneticEncoder*);
  DCMotor(MagneticEncoder*, uint8_t);
  void CW(uint8_t);
  void CCW(uint8_t);
  void Stop();
  void PWM(uint8_t, int, int);

  char* GetEncoderTopicName();
  double GetEncoderAngle();
};

//////////////////////////////////////////////////////////////////

DCMotor::DCMotor()
  : DCMotor::DCMotor((MagneticEncoder *)NULL) {}

DCMotor::DCMotor(uint8_t direction) 
  : DCMotor::DCMotor(13, 12, NULL, direction) {}

DCMotor::DCMotor(int INL, int INH) 
  : DCMotor::DCMotor(INL, INH, (MagneticEncoder*)NULL) {}

DCMotor::DCMotor(int INL, int INH, uint8_t direction) 
  : DCMotor::DCMotor(INL, INH, NULL, direction) {}

DCMotor::DCMotor(MagneticEncoder* encoder) 
  : DCMotor::DCMotor(13, 12, encoder, "RIGHT") {}

DCMotor::DCMotor(int INL, int INH, MagneticEncoder* encoder)
  : DCMotor::DCMotor(INL, INH, encoder, "RIGHT") {}

DCMotor::DCMotor(MagneticEncoder* encoder, uint8_t direction)
  : DCMotor::DCMotor(13, 12, encoder, direction) {}

DCMotor::DCMotor(int INL, int INH, MagneticEncoder* encoder, 
                                      uint8_t direction)
  : INL(INL), 
    INH(INH),
    encoder(encoder),
    direction(direction) {

      DCMotor::initPins();
    }

//////////////////////////////////////////////////////////////////

void DCMotor::initPins() {

  pinMode(this->INH, OUTPUT);
  pinMode(this->INL, OUTPUT);
  DCMotor::Stop();
}

void DCMotor::CW(uint8_t velocity) {
  // Motor gira en un sentido
  if (this->direction == LEFT) 
  {
    DCMotor::PWM(velocity, this->INH, this->INL);
  } else if (this->direction == RIGHT)
  {
    DCMotor::PWM(velocity, this->INL, this->INH);
  } else 
  {
    DCMotor::PWM(LOW, this->INH, this->INL);
  }
}

void DCMotor::Stop() {
  // Motor no gira
  DCMotor::PWM(LOW, this->INH, this->INL);
}

void DCMotor::CCW(uint8_t velocity) {

  // Motor gira en sentido inverso
  if (this->direction == LEFT) 
  {
    DCMotor::PWM(velocity, this->INL, this->INH);
  } else if (this->direction == RIGHT)
  {
    DCMotor::PWM(velocity, this->INH, this->INL);
  } else 
  {
    DCMotor::PWM(LOW, this->INH, this->INL);
  }
}

void DCMotor::PWM(uint8_t velocity, int pwm_pin, int low_pin) {	
	// For security reasons
  velocity > MAX_VALUE? velocity = MAX_VALUE : velocity;

	digitalWrite(low_pin, LOW);
	analogWrite(pwm_pin, velocity);
}

char* DCMotor::GetEncoderTopicName() {

  return encoder->GetTopicName();
}

double DCMotor::GetEncoderAngle() {

  return encoder->GetAngle();
}
