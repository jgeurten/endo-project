#include "configuration.h"
#include "GCodeInterpreter.h"
#include "Laser.h"

#include <stdlib.h> 
#include <Arduino.h>

//char serialMsg[MAXBYTES + 1]; //extra block of memory for '\0\'
int numBytes;

void processCode(char* msg) {

	//int GCode = parseMessage(msg[0%4]);
int GCode = parseMessage(msg);
	//check if empty or null or does not start with 'G'
	if (GCode == 0 || GCode == -1) return;

	switch (GCode) {
	case 11:
		initializeLaser();
		break;
	case 21:
		laserOn();
		break;
	case 32:
		laserOff();
		break;
	case 35:
		//function21();
		break;
	case 72:
		//function22();
		break;
	default:
		break;
	}
}

int parseMessage(char* msg)
{
	//Input the serial char message from Nano and output the integer code
	if (msg[0] == 'G') {
		char temp[MAXBYTES] = { msg[1], msg[2], msg[3] };
		int code = atoi(temp);
		return code;
	}
	else
		return -1;
}

void help() {

	Serial.println("Endo Scanner G-Code Version:");
	displayVersion();
	Serial.println("Commands:");
	Serial.println("G11; - do this");
	Serial.println("G22; - do that");
	Serial.println("G33; - do this and that"); //etc.

	Serial.println("G51 - PWM 50%");
}

void displayVersion() {

	Serial.println(VERSION);
}

void reset() {

	//Ready to receive input from serial line - intialize buffer array
	//serialMsg[] = { 0 };
}
