#pragma once

//#define _16BIT_PWM_

#ifdef _16BIT_PWM_
  #define INT_PWM   uint16_t
  const INT_PWM MAX_VALUE PROGMEM = 0xFFFF;
#else
  #define INT_PWM   uint8_t
  const INT_PWM MAX_VALUE PROGMEM = 0xFF;
#endif

//////////////////////////////////////////////////////////////////

// Magnetic encoder's signals

#define CS1	6
#define CS2	7

// DC motor's signals

#define IN1	11
#define IN2 10
#define IN3 9
#define IN4	8
