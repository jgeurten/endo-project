#include "Laser.h";
#include "configuration.h"
#include "GCodeInterpreter.h"

char serialMsg; //extra block of memory for '\0\'
int count = 0;
char first[MAXBYTES + 1];
byte msg[2] = {0xFF, 0xAA};
bool echoed = false;

 
void setup() 
{

  //initializeLaser();   // sets baud rate as well.
 // interruptsOn();
 Serial.begin(9600); 
 // attachInterrupt(digitalPinToInterrupt(ISR_PIN), buttonPress, CHANGE);
 pinMode(2, OUTPUT); 
 digitalWrite(2, HIGH); //change

}

void loop()
{
  if(Serial.available() > 0)
  {
    serialMsg = Serial.read(); 
    if(serialMsg == '1')
     {
       Serial.print("ON"); 
       digitalWrite(2, HIGH); 
       
     }
     if(serialMsg =='2')
     {
       Serial.print("OFF"); 
       digitalWrite(2, LOW); 
     }
  }
}

