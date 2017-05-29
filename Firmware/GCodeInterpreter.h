#pragma once
#ifndef GCODEINTERPRETER_H
#define GCODEINTERPRETER_H

#include "Arduino.h"


char buffer[MAX_BUF];
int buffSize;

void help();
void displayVersion();
void processCode();
void reset();

#endif // !GCODEINTERPRETER_H
