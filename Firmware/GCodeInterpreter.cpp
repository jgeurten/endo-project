#include "configuration.h"
#include "GCodeInterpreter.h"


void processCode(char message) {

	//check if empty or null
	if (message == NULL) return;
	int cmd;
	cmd = parsecode(message);

	switch (cmd) {
	case 0:
		//function0()
		break;
	case 1:
		//function1();
		break;
	case 2:
		//function2();
		break;
	case 20:
		//function20();
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

int parsecode(char message) {
	

	return atoi(message);	
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
	buffSize = 0;
	buffer[0] = ';';
	Serial.println(">");
}