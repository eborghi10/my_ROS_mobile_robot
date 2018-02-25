#pragma once

#include "MagneticEncoder.h"

class DCMotor {

  MagneticEncoder *encoder;
  ros::Subscriber<std_msgs::Float32, DCMotor> sub;

  const uint8_t INL; 
  const uint8_t INH;
  const uint8_t CH_INH;
  const uint8_t CH_INL;
  const double frequency;
  const uint8_t resolutionBits;
  const char* name;

  void initPins();
  void Stop();
  void CW(float);
  void CCW(float);
  int32_t mapToInt(float);

  void motorCb(const std_msgs::Float32&);

public:

  DCMotor();
  DCMotor(const uint8_t,const uint8_t);
  DCMotor(MagneticEncoder*);
  DCMotor(const uint8_t,const uint8_t,MagneticEncoder*);
  DCMotor(const uint8_t,const uint8_t,MagneticEncoder*,const char*);

  void PublishAngle();
};

DCMotor::DCMotor(const uint8_t INL, const uint8_t INH, MagneticEncoder* encoder, const char* name)
  : INL(INL)
  , INH(INH)
  , CH_INL(0)
  , CH_INH(1)
  , frequency(5000) // 5 kHz
  , resolutionBits(16)
  , encoder(encoder)
  , name(name)
  , sub(strcmp(name, "left") == 0 ? "my_robot/left_wheel_vel" 
                : "my_robot/right_wheel_vel", &DCMotor::motorCb, this)
{
    DCMotor::initPins();
    nh.subscribe(sub);
}

void DCMotor::initPins() {

  // Setup channel CH_INH
  ledcSetup(this->CH_INH, this->frequency, this->resolutionBits);
  // Attach pin INH to channel CH_INH
  ledcAttachPin(this->INH, this->CH_INH);

  ledcSetup(this->CH_INL, this->frequency, this->resolutionBits);
  ledcAttachPin(this->INL, this->CH_INL);
  
  DCMotor::Stop();
}

void DCMotor::Stop() {
  // Stop motor

  // Initialize channels to off
  ledcWrite(this->INL, LOW);
  ledcWrite(this->INH, LOW);
}

void DCMotor::CW(float val) {
  // Motor turns forward or CW
  ledcWrite(this->INL, LOW);
  ledcWrite(this->INH, DCMotor::mapToInt(val));
}

void DCMotor::CCW(float val) {
  // Motor turns in the inverse direction or CCW
  ledcWrite (this->INL, DCMotor::mapToInt(val));
  ledcWrite (this->INH, LOW);
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
int32_t DCMotor::mapToInt(float val) {
  int32_t maxValue = pow(2, this->resolutionBits) - 1; // (2 ^ N)-1
  return map(static_cast<int32_t>(val), 0, 1, 0, maxValue);
}
