#include "GCodeInterpreter.h"
#include "configuration.h"
#include "Laser.h"
#include "Arduino.h"


bool isLaserOn;

long prevTime = 0;
long currTime = 0;

volatile byte laserState;

void initializeLaser()
{
	Serial.begin(BAUD_RATE);
	
	pinMode(LASER_ENABLE_PIN, OUTPUT);
	digitalWrite(LASER_ENABLE_PIN, LOW);

	isLaserOn = false;
	laserState = LOW;
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


void laserButtonPress()
{
	currTime = millis();
	if (currTime - prevTime > ISRdelay) {
		if (isLaserOn) laserOff();
		else laserOn();

		prevTime = currTime;
	}
	else
		return;
}
