// Include libraries
#include "include/ESP32HW.h"
#include "include/DCMotor.h"
#include "include/MagneticEncoder.h"

DCMotor *motor_left;
DCMotor *motor_right;

void setup()
{
    Serial.begin(115200);
    setupWiFi();
    // TODO: change to while(!connected) > keepReconnecting()
    delay(2000);
    nh.initNode();

    motor_left = new DCMotor(IN1, IN2, 
                 new MagneticEncoder(CS1, LEFT),
                 (char*)"left");

    motor_right = new DCMotor(IN3, IN4,
                  new MagneticEncoder(CS2, RIGHT),
                  (char*)"right");
}

void loop() 
{
    nh.spinOnce();

    motor_left->PublishAngle();
    motor_right->PublishAngle();

    delay(50);
}
