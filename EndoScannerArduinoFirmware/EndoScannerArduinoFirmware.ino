#include "configuration.h"
#include "Laser.h"
#include "GCodeInterpreter.h"
char serialMsg; //extra block of memory for '\0\'
int count = 0;
bool scanning = false; 
bool paused = false; 
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
 pinMode(4, OUTPUT); 
 digitalWrite(4, LOW); //change

 attachInterrupt(digitalPinToInterrupt(3), goIsr, RISING);
 attachInterrupt(digitalPinToInterrupt(2), stopIsr, RISING);

}
void goIsr()
{
  if(!scanning)
  {
    scanning = true; 
    Serial.println("GO"); 
  }
}

void stopIsr()
{
  if(scanning && !paused)
  {
    paused = true; 
    Serial.println("PS"); 
  }

  if(scanning && paused)
  {
    scanning = false; 
    Serial.println("DN"); 
  }
}
void loop()
{
  if(Serial.available() > 0)
  {
    serialMsg = Serial.read(); 
    if(serialMsg == '1')
     {
       Serial.print("ON"); 
       digitalWrite(4, LOW); 
       
     }
     if(serialMsg =='2')
     {
       Serial.print("OFF"); 
       digitalWrite(4, HIGH); 
     }
  }
}

