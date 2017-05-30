#pragma once
#ifndef GCODEINTERPRETER_H
#define GCODEINTERPRETER_H

//#include "Arduino.h"


int parseMessage(char *msg);
void help();
void displayVersion();
void processCode(char *msg);
void reset();

#endif // !GCODEINTERPRETER_H
