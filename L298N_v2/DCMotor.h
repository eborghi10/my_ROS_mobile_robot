#pragma once

class DCMotor {
public:
  DCMotor();
  DCMotor(int,int);
  void CW();
  void CCW();
  void Stop();
  void PWM(uint16_t);
private:  
  int INL; 
  int INH;
  void initPins();
};

DCMotor::DCMotor()
  : INL(5), INH(4) {
  	DCMotor::initPins();
  }

DCMotor::DCMotor(int INL, int INH) {
  this->INL = INL;
  this->INH = INH;
  DCMotor::initPins();
}

void DCMotor::initPins() {
	pinMode(this->INH, OUTPUT);
  	pinMode(this->INL, OUTPUT);
}

void DCMotor::CW() {
  // Motor gira en un sentido
  digitalWrite (INL, LOW); 
  digitalWrite (INH, HIGH);
}

void DCMotor::Stop() {
  // Motor no gira
  digitalWrite (INL, LOW); 
  digitalWrite (INH, LOW); 
}

void DCMotor::CCW() {
  // Motor gira en sentido inverso
  digitalWrite (INL, HIGH);
  digitalWrite (INH, LOW);
}

void DCMotor::PWM(uint16_t val) {
	// For security reasons
	val > 255? val = 255 : val;
	
	digitalWrite(INL, LOW);
	analogWrite(INH, val);
}