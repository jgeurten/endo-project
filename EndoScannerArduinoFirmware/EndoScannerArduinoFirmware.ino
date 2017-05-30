#include "Laser.h";
#include "configuration.h"
#include "GCodeInterpreter.h"

 char serialMsg[MAXBYTES + 1]; //extra block of memory for '\0\'

void setup() {
  initializeLaser();   // sets baud rate as well.
  //reset();
}

void loop()
{
  while (Serial.available() > 0)
  {
    delay(100);
   
    for (int numBytes = 0; numBytes < MAXBYTES; numBytes++) {

      serialMsg[numBytes] = Serial.read();
      serialMsg[numBytes + 1] = '\0';    //serialMsg[1] = '\0'
    }
    //Serial.print(serialMsg);
    processCode(serialMsg);
  }
}




