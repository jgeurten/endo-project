/*
 * #include "Laser.h";
 *#include "configuration.h"
 *#include "GCodeInterpreter.h"
 * 
 */

char serialMsg[2]; //extra block of memory for '\0\'
int numBytes;
int code;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  //reset();
}

void processCommand(int code)
{
  switch (code) {
    case 1:
      digitalWrite(LED_BUILTIN, LOW);
      break;
      case 2:
     digitalWrite(LED_BUILTIN, LOW);
     break;
    case 9:
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    default:
      break;
  }
}
void loop()
{
  while (Serial.available() >0)
  {
    delay(100);
    if(numBytes != 1) {

      serialMsg[numBytes] = Serial.read();
      numBytes++;
      serialMsg[numBytes] = '\0';    //serialMsg[1] = '\0'
    }
    code = atoi(serialMsg); //last byte must be null
    Serial.print(code);
   processCommand(code);
   //if(code ==9)  digitalWrite(LED_BUILTIN, HIGH);
   //if(code ==1)  digitalWrite(LED_BUILTIN, LOW);
    numBytes = 0;
  }

}




