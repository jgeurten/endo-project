volatile bool enable = false;
long prevTime = 0;
long currTime;
int ISRdelay = 1500;
volatile byte laserState = LOW;

void setup() {
  // put your setup code here, to run once:

  pinMode(9, OUTPUT);
  pinMode(2, INPUT);
  pinMode(12, OUTPUT);
  digitalWrite(12,  HIGH);
  Serial.begin(9600);

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

  attachInterrupt(digitalPinToInterrupt(2), buttonPress, CHANGE);

}

void loop() {

  if (enable) {
    digitalWrite(9, laserState);
  }
  else {
    digitalWrite(9, LOW);
  }
}

void buttonPress() {

  currTime = millis();
  if (currTime - prevTime > ISRdelay) {
    enable = !enable;
    prevTime = currTime;
    //Serial.print("In ISR");
  }
}

ISR(TIMER1_COMPA_vect) {
  laserState = !laserState;
}

