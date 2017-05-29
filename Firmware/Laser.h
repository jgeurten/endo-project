#pragma once

#ifndef LASER_H
#define LASER_H

#include "Arduino.h"

void initializeLaser();
void laserOn();
void laserOff();
bool isLaserOn();
void laserPWM();

#endif // !LASER_H
