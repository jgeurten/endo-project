#include "GCodeInterpreter.h"
#include "configuration.h"
#include "Laser.h"
#include "Arduino.h"

void initializeLaser()
{
	Serial.begin(BAUD_RATE);
	pinMode(LASER_ENABLE_PIN, OUTPUT);
	pinMode(LASER_PWM_PIN, OUTPUT);
	digitalWrite(LASER_ENABLE_PIN, LOW);
	digitalWrite(LASER_PWM_PIN, LOW);
	laserOn = false;
}

void laserOn()
{
	if (!laserOn) {
		digitalWrite(LASER_ENABLE_PIN, HIGH);
		laserOn = true;
	}
	else
		return;
}

void laserOff()
{
	if (laserOn) {
		digitalWrite(LASER_ENABLE_PIN, LOW);
		laserOn = false;
	}
	else
		return;
}

void laserPWM()
{
	//too add: using interrupt service routine of clock 
}

bool isLaserOn() {

	return laserOn;
}