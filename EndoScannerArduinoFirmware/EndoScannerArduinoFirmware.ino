#include "configuration.h"
#include "gCodeInterpretter.h"

void setup() {
  Serial.begin(BAUD_RATE);
  displayVersion();
  intializeLaser();//turn on quickly - check if turned on
  reset(); //initialize buffSize and buffer[]
}

void loop() {

  while (Serial.available() > 0)
  {
    string msg = Serial.read();
    Serial.print(msg);
    if (buffSize < MAX_BUF - 1)
      message[buffSize + 1] = msg;
    if (msg == '\n' || msg == '\r') { //ensure message is fully received
      buffer(buffSize) = msg;
      processCode();
      reset();
    }

  }

}
