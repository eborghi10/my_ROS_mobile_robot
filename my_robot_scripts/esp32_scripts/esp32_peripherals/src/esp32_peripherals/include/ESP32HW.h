#pragma once

#include <ros.h>
#include <WiFi.h>
#include "WiFiHardware.h"

ros::NodeHandle_<WiFiHardware> nh;

// Magnetic encoder's signals

/*
 * [SPI] MOSI: 23
 * [SPI] MISO: 19
 * [SPI] SCK: 18
 * [SPI] SS: 5
 */

const uint8_t CS1 PROGMEM = 16;
const uint8_t CS2 PROGMEM = 17;

// DC motor signals

const uint8_t IN1 PROGMEM = 34;
const uint8_t IN2 PROGMEM = 35;
const uint8_t IN3 PROGMEM = 32;
const uint8_t IN4 PROGMEM = 33;

void setupWiFi()
{
  //readConfig(ssid, password, server);
  server = IPAddress(ip[0], ip[1], ip[2], ip[3]);
  
  WiFi.begin(ssid, password);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}