#pragma once

#include <ros.h>

ros::NodeHandle nh;

// Magnetic encoder's signals

const uint8_t CS1 PROGMEM = 6;
const uint8_t CS2 PROGMEM = 7;

// DC motor signals

const uint8_t IN1 PROGMEM = 11;
const uint8_t IN2 PROGMEM = 10;
const uint8_t IN3 PROGMEM = 9;
const uint8_t IN4 PROGMEM = 8;

//#define _16BIT_PWM_

#ifdef _16BIT_PWM_
  #define INT_PWM   uint16_t
  const INT_PWM MAX_VALUE PROGMEM = 0xFFFF;
#else
  #define INT_PWM   uint8_t
  const INT_PWM MAX_VALUE PROGMEM = 0xFF;
#endif