#include "include/MagneticEncoder.h"
#include "include/WiFiHardware.h"

// Encoder pins for ESP32
#define CS1	16
#define CS2	17

MagneticEncoder *encoder_left;
MagneticEncoder *encoder_right;

void setup()
{	
	Serial.begin(115200);
	if(!setupWiFi())
	{
		Serial.println("Error connecting to WiFi");
		return;
	}
	delay(2000);
	encoder_left = new MagneticEncoder(CS1, LEFT);
	encoder_right = new MagneticEncoder(CS2, RIGHT);
}

void loop()
{
	(encoder_left->nh).spinOnce();
	(encoder_right->nh).spinOnce();

	encoder_left->PublishAngle();
	encoder_right->PublishAngle();
	
	delay(50);
}
