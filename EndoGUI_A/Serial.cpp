//EndoSerial.cpp

//Locals includes
#include "Serial.h"

//MSDN includes
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

using namespace std;

Serial::Serial(std::string portName)
{
	//try to connect serial port
	this->connected = false;

	//use create file function to establish serial connection
		
	this->hSerial = CreateFile(portName.c_str(),
		GENERIC_WRITE,					//desired connection = write only (generic_read)
		0,								//sharemode = 0: prevents other processes from connecting
		NULL,							
		OPEN_EXISTING,					//opens file only if it exists
		FILE_ATTRIBUTE_NORMAL,
		NULL);

	if (this->hSerial != "INVALID_HANDLE_VALUE")	//port exists and connected
	{
			
		// set comm parameters:
		DCB dcbParams = { 0 };

		dcbParams.BaudRate = BAUD_RATE;
		dcbParams.Parity = NOPARITY;
		dcbParams.StopBits = ONESTOPBIT;
		dcbParams.ByteSize = 8;

		dcbParams.fDtrControl = DTR_CONTROL_ENABLE;	//resets Arduino upon connection establishment

		if (!SetCommState(hSerial, &dcbParams))
		{
			Serial::~Serial();
			return;
			throw("ERROR: Could not set com port parameters");
		}
		else {
			//connected to the port
			this->connected = true;
			this->portName = portName;
			this->baud = BAUD_RATE;
			PurgeComm(this->hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);

			Sleep(WAIT_TIME);
		}
	}
}

Serial::~Serial()
{
	if (this->connected)
	{
		this->connected = false;
		CloseHandle(this->hSerial);
	}
}

bool Serial::write(char message)	
{
	DWORD numBytesWritten; 
	WriteFile(hSerial, &message, message, &numBytesWritten, NULL); //3rd argument set to NULL if asynchronous
	if (numBytesWritten > 0) return true;
	else return false;
}

bool Serial::isConnected()
{
	return this->connected;
}