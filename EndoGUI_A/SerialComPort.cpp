//EndoSerial.cpp

#include <QSerialPort>
#include <qserialportinfo.h>
//Locals includes
#include "SerialComPort.h"

//MSDN includes
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <qdebug.h>

using namespace std;

Serial::Serial(std::string portName)
{
	//try to connect serial port
	this->connected = false;

	//use create file function to establish serial connection
	
	QString serialPortName = QString::fromStdString(portName);

	this->serialPort.setPortName(serialPortName);
	this->serialPort.setBaudRate(BAUD_RATE);
	this->serialPort.setDataBits(QSerialPort::Data8);
	this->serialPort.setParity(QSerialPort::NoParity);
	this->serialPort.setStopBits(QSerialPort::OneStop);
	this->serialPort.setFlowControl(QSerialPort::NoFlowControl);

	if (this->serialPort.open(QIODevice::ReadWrite))	//open & readable and writeable
	{
		this->connected = true;
		this->serialPort.clear(QSerialPort::AllDirections);
		Sleep(WAIT_TIMEOUT);
	}
	else
	{
		ERROR("Could not connect to com port:", portName);
		return;
	}
}

Serial::~Serial()
{
	if (this->connected)
		this->connected = false;
}

bool Serial::write(string message)
{
	qint64 bytes;
	serialPort.write(message.c_str());
	serialPort.flush();
	serialPort.bytesWritten(bytes);
	if (bytes == BUFSIZE)
		return true;
	else
		return false;
}

string Serial::read()
{
	return "notcalled";
}

bool Serial::isConnected()
{
	return this->connected;
}


