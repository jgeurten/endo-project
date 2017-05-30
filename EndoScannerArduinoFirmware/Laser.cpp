#include "GCodeInterpreter.h"
#include "configuration.h"
#include "Laser.h"
#include "Arduino.h"

bool isLaserOn;

void initializeLaser()
{
	Serial.begin(BAUD_RATE);
	pinMode(LASER_ENABLE_PIN, OUTPUT);
	pinMode(LASER_PWM_PIN, OUTPUT);
	digitalWrite(LASER_ENABLE_PIN, LOW);
	digitalWrite(LASER_PWM_PIN, LOW);
	isLaserOn = false;
}

void laserOn()
{
	if (!isLaserOn) {
		digitalWrite(LASER_ENABLE_PIN, HIGH);
		isLaserOn = true;
	}
	else
		return;
}

void laserOff()
{
	if (isLaserOn) {
		digitalWrite(LASER_ENABLE_PIN, LOW);
		isLaserOn = false;
	}
	else
		return;
}

void laserPWM()
{
	//too add: using interrupt service routine of clock 
}

