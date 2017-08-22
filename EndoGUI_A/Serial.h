/*#pragma once
#ifndef SERIAL_H
#define SERIAL_H

#include <QSerialPort>
#include "Q"

#include <string>
#include "MainWindow.h"

 class Serial
{
public:
	Serial(std::string portName);
	~Serial();

private:
	void write(string message);
	string read(); 

	bool connected; 
	int baudRate = 9600;
};

Serial::Serial(std::string portName)
{
	this->connected = false;
	QSerialPort serialPort; 
	serialPort.setPortName(portName);
	serialPort.setBaudRate(baudRate);
	serialPort.setReadBufferSize(3);
	serialPort.parity(0);			//no praity 
	
	//Attempt connection
	if (!serialPort.open(QIODevice::ReadWrite)) {
		throw("ERROR: Unable to open serial port");
		
	}

	this->connected = true; 

}

Serial::~Serial()
{
	~QSerialPort(); 
}
#endif // !SERIAL_H

*/