#pragma once

#include <ros.h>
#include <robot_msgs/Arduino.h>

ros::NodeHandle nh;

// Magnetic encoder's signals

#define CS1	6
#define CS2	7

// DC motor's signals

#define IN1	11
#define IN2 10
#define IN3 9
#define IN4	8

///////////////////////////////////////

const boolean LEFT PROGMEM = false;
const boolean RIGHT PROGMEM = true;