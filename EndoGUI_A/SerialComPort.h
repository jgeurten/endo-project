#pragma once
#pragma once
#ifndef SERIALCOMPORT_H
#define SERIALCOMPORT_H

#include <QtSerialPort\qserialport.h>

#include <string>
#include "windows.h"


#define BAUD_RATE 115200
#define WAIT_TIME 2000
#define BUFSIZE   3

using namespace std;

class Serial
{
private:

	QSerialPort serialPort;
	bool connected;
	string portName;
	
public:
	Serial(string portName);
	~Serial();
	bool write(string message);
	bool isConnected();
	string read();
};
#endif // !SERIAL_H