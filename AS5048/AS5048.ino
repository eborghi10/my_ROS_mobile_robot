/**
 * Library issue:
 * https://github.com/ZoetropeLabs/AS5048A-Arduino/issues/3
 *
 * Other lib:
 * http://forum.arduino.cc/index.php?topic=155238.0
 *
 */

#include <SPI.h>	// Throws an error in Windows
#include <Streaming.h>
#include <AS5048A.h>

AS5048A angleSensor(10);
uint16_t zero_position;

float inline mapeo(uint16_t reg_val) {
	return reg_val * ((float)360 / 65535);
};

float inline mapeo2(float reg_val) {
	return reg_val - 180;
}

void setup()
{
	Serial.begin(115200);
	angleSensor.init();
//	zero_position = angleSensor.getZeroPosition();
	zero_position = angleSensor.getRotation();
	Serial << "> Zero: " << zero_position << endl;
}

void loop()
{
	delay(1000);

	uint16_t rel_angle = angleSensor.getRotation();
	float rel_angle_map = mapeo(rel_angle - zero_position);
	float rel_angle_map2 = mapeo2(rel_angle_map - zero_position);
//	Serial << "> Rel: " << rel_angle
//			<< "\t" << _BIN(rel_angle) 
//			<< "\t" << _HEX(rel_angle) 
//			<< "\t" << (unsigned int)rel_angle << endl;
	Serial << rel_angle_map << "\t" << rel_angle_map2 << endl;
//	Serial << rel_angle << endl;

//	uint16_t abs_angle = angleSensor.getRawRotation();
//	uint16_t map_angle = map(abs_angle,0,65535,0,360);
//	Serial << "> Raw: " << abs_angle 
//			<< "\t" << _BIN(abs_angle) 
//			<< "\t" << _HEX(abs_angle) << endl;
//	Serial << map_angle << endl;

//	Serial << "> Raw-Zero: " 
//			<< abs_angle - zero_position << endl;

//	Serial << "> Hex: " << _HEX(abs_angle) << endl;

//	Serial << endl << "---------------------" << endl << endl;

	if (angleSensor.error()) {
		Serial << "ERROR: " << angleSensor.getErrors() << endl;
	}
}