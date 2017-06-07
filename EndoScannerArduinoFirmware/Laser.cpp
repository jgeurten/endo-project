#include "GCodeInterpreter.h"
#include "configuration.h"
#include "Laser.h"
#include "Arduino.h"


bool isLaserOn;

long prevTime = 0;
long currTime = 0;

volatile byte laserState;
volatile bool pwmEnabled;

void initializeLaser()
{
	Serial.begin(BAUD_RATE);
	
	pinMode(LASER_ENABLE_PIN, OUTPUT);
	pinMode(LASER_PWM_PIN, OUTPUT);
 
	digitalWrite(LASER_ENABLE_PIN, LOW);
	digitalWrite(LASER_PWM_PIN, LOW);

	isLaserOn = false;
	pwmEnabled = false;
	laserState = LOW;

}

void laserOn()
{
	if (!isLaserOn && !pwmEnabled) {
		digitalWrite(LASER_ENABLE_PIN, HIGH);
		isLaserOn = true;
	}
	else
		return;
}

void laserOff()
{
	if (isLaserOn && !pwmEnabled) {
		digitalWrite(LASER_ENABLE_PIN, LOW);
		isLaserOn = false;
	}
	else
		return;
}

void laserPWM()
{
	if (pwmEnabled)
	{
		laserState = !laserState;
		laserState = HIGH;  //for constant laser line
		digitalWrite(LASER_ENABLE_PIN, laserState);
	}
	else
		return;
}

void PWMButtonPress()
{
	pwmEnabled = !pwmEnabled;
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