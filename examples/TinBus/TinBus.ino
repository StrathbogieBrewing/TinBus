#include "TinBus.h"

#define kRxInterruptPin (2)
#define kBaudRate (1200L)
void tinbusCallback(tinframe_t *frame);
TinBus tinBus(Serial, kBaudRate, kRxInterruptPin, tinbusCallback);

void setup() {
  tinBus.begin();
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  static unsigned long secondsTimer = 0;

  tinBus.update();

  unsigned long m = millis();
  if(m - secondsTimer > 1000L){
    secondsTimer = m;

    tinframe_t txFrame;
    msg_t *txMsg = (msg_t *)txFrame.data;
    strcpy(txMsg, "Hello!!");  // 10 bytes maximum payload
    tinBus.write(&txFrame);
  }


  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}

void tinbusCallback(tinframe_t *frame){

}
