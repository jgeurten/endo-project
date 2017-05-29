#pragma once
#ifndef SERIAL_H
#define SERIAL_H

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>

#define BAUD_RATE 115200
#define WAIT_TIME 2000

Class Serial
{
private:

	bool connected;
	string comName;
	int baud;
	HANDLE hSerial;

public:
	Serial();
	~Serial();
	bool WriteData();
	bool portConnected();
}
#endif // !SERIAL_H
