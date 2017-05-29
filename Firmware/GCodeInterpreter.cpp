#include "configuration.h"
#include "GCodeInterpreter.h"
#include "Laser.h"

char serialMsg[maxByteNum + 1]; //extra block of memory for '\0\'
int numBytes;
int code;

void processCode(int code) {

	//check if empty or null
	if (code == NULL) return;
	
	switch (code) {
	
	case 1:
		initializeLaser();
		break;
	case 2:
		laserOn();
		break;
	case 20:
		laserOff();
		break;
	case 21:
		//function21();
		break;
	case 22:
		//function22();
		break;
	default:
		break;
	}
}




void help() {

	Serial.println("Endo Scanner G-Code Version:");
	displayVersion();
	Serial.println("Commands:");
	Serial.println("G01; - do this");
	Serial.println("G02; - do that");
	Serial.println("G03; - do this and that"); //etc.

	Serial.println("G50 - PWM 50%");
}

void displayVersion() {

	Serial.println(VERSION);
}

void reset() {

	//Ready to receive input from serial line - intialize buffer array
	numBytes = 0;
	serialMessage = { 0 };
}