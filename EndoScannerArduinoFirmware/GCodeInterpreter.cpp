#include "configuration.h"
#include "GCodeInterpreter.h"
#include "Laser.h"

#include <stdlib.h>
#include <Arduino.h>


void interruptsOn()
{
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 34286; // for 1Hz
  //OCR1A = 17143;// for 2Hz

  TCCR1B |= (1 << WGM12); //Clear on Timer Compare (CTC) mode
  TCCR1B |= (1 << CS12); //prescalar = 256 - timer frq = 62500 Hz
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();

}
void processCode(char* msg) {

  //int GCode = parseMessage(msg[0%4]);
  int GCode = parseMessage(msg);
  //Serial.print(GCode);
  //check if empty or null or does not start with 'G'
  if (GCode == 0 || GCode == -1) return;

  switch (GCode) {
    case 11:
      initializeLaser();
      break;
    case 21:
      laserOn();
      break;
    case 32:
      laserOff();
      break;
    case 35:
      //function21();
      break;
    case 72:
      //function22();
      break;
    default:
      break;
  }
}

int parseMessage(char* msg)
{
  if (msg[0] == 'G') {
    char temp[MAXBYTES] = { msg[1], msg[2], msg[3] };
    int code = atoi(temp);
    return code;
  }
  else
    return -1;
}

void help() {

  Serial.println("Endo Scanner G-Code Version:");
  displayVersion();
  Serial.println("Commands:");
  Serial.println("G11; - do this");
  Serial.println("G22; - do that");
  Serial.println("G33; - do this and that"); //etc.

  Serial.println("G51 - PWM 50%");
}

void displayVersion() {

  Serial.println(VERSION);
}

void reset() {

  //Ready to receive input from serial line - intialize buffer array
  //serialMsg[] = { 0 };
}
