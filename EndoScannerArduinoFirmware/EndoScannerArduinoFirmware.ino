#include "Laser.h";
#include "configuration.h"
#include "GCodeInterpreter.h"

char serialMsg[MAXBYTES + 1]; //extra block of memory for '\0\'
int count = 0;
char first[MAXBYTES + 1];

void setup() {
 
  initializeLaser();   // sets baud rate as well.
  interruptsOn();
  attachInterrupt(digitalPinToInterrupt(ISR_PIN), buttonPress, CHANGE);
  
}

void loop()
{
  if (Serial.available() > 0)
  {
    delay(100);
    for (int numBytes = 0; numBytes < MAXBYTES; numBytes++) {
      serialMsg[numBytes] = Serial.read();
      serialMsg[numBytes + 1] = '\0';    //serialMsg[1] = '\0'
    }
    processCode(serialMsg);
  }
}

void buttonPress() {
  laserButtonPress();
}

ISR(TIMER1_COMPA_vect) {
  laserPWM();
}
