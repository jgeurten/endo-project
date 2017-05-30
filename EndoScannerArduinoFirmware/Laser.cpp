#include "GCodeInterpreter.h"
#include "configuration.h"
#include "Laser.h"
#include "Arduino.h"

bool isLaserOn;

void initializeLaser()
{
	Serial.begin(BAUD_RATE);
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(LASER_ENABLE_PIN, OUTPUT);
	pinMode(LASER_PWM_PIN, OUTPUT);
	digitalWrite(LASER_ENABLE_PIN, LOW);
	digitalWrite(LASER_PWM_PIN, LOW);
	isLaserOn = false;
  //digitalWrite(LED_BUILTIN, HIGH);
}

void laserOn()
{
	if (!isLaserOn) {
		//digitalWrite(LASER_ENABLE_PIN, HIGH);
		digitalWrite(LED_BUILTIN, HIGH);
		isLaserOn = true;
	}
	else
		return;
}

void laserOff()
{
	if (isLaserOn) {
		digitalWrite(LED_BUILTIN, LOW);
		isLaserOn = false;
	}
	else
		return;
}

void laserPWM()
{
	//too add: using interrupt service routine of clock 
}

