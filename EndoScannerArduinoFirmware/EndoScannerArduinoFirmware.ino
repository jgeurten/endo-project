#include "configuration.h"
#include "Laser.h"
#include "GCodeInterpreter.h"
char serialMsg; //extra block of memory for '\0\'
int count = 0;

byte msg[2] = {0xFF, 0xAA};
bool echoed = false;

 
void setup() 
{
pinMode(0, OUTPUT); 
digitalWrite(0, HIGH);
  //initializeLaser();   // sets baud rate as well.
 // interruptsOn();
 Serial.begin(9600); 
 // attachInterrupt(digitalPinToInterrupt(ISR_PIN), buttonPress, CHANGE);
 pinMode(1, OUTPUT); 
 digitalWrite(1, LOW); //change

}

void loop()
{
  if(Serial.available() > 0)
  {
    serialMsg = Serial.read(); 
    if(serialMsg == '1')
     {
       Serial.print("ON"); 
       digitalWrite(1, HIGH); 
       
     }
     if(serialMsg =='2')
     {
       Serial.print("OFF"); 
       digitalWrite(1, LOW); 
     }
  }
}

