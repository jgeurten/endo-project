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
	string message;
	int baud;
	HANDLE hSerial;
	DWORD numBytesToRead = 3;
	char buffer[3];
	OVERLAPPED overlapped = { 0 };

public:
	Serial(string portName);
	~Serial();
	bool write(string message);
	bool isConnected();
	string read();
	void CALLBACK ReadFileCompleted(const DWORD errorCode,
		const DWORD bytesCopied,
		OVERLAPPED* overlapped);
};
#endif // !SERIAL_H