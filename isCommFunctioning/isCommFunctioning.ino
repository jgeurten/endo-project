#include "testheader.h"

bool ledState = HIGH;
char msg;
int code;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {

while(Serial.available()){
  msg = Serial.read();
  code = atoi(msg);
  if (code == 9 ){
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN,HIGH);
  
  }

  if (code ==0){
    digitalWrite(LED_BUILTIN, LOW);
  }

  Serial.println(code);
}
}
