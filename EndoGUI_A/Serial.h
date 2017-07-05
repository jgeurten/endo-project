#pragma once
#ifndef SERIAL_H
#define SERIAL_H

#include <string>
#include "windows.h"

#define BAUD_RATE 115200
#define WAIT_TIME 2000

using namespace std;

class Serial
{
private:

	bool connected;
	string portName;
	int baud;
	HANDLE hSerial;

public:
	Serial(string portName);
	~Serial();
	bool write(string message);
	bool isConnected();
	
};
#endif // !SERIAL_H