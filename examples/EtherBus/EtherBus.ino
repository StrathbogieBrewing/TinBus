// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM
//      (D 8) PB0 14|    |15  PB1 (D 9) PWM
//                  +----+

#include "TinBus.h"

#define PIN_LED_AMBER (9)

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(PIN_LED_AMBER, OUTPUT);
  tinbusBegin();
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {

delayMicroseconds(5000);

  int16_t rxData = tinbusRead();
  if(rxData >= 0){
    Serial.println(rxData, HEX);
  } else
  if(rxData == TINBUS_END_OF_FRAME){
    Serial.println("EOF");
  } else if(rxData != TINBUS_NO_DATA){
    Serial.println(rxData);
  }

  // PORTB |= (1 << 1);
  // PORTB &= ~(1 << 1);

  // static uint8_t data = 0;
  //
  // // digitalWrite(PIN_LED_AMBER, HIGH);   // turn the LED on (HIGH is the voltage level)
  // // delayMicroseconds(10000);                      // wait for a second
  // // digitalWrite(PIN_LED_AMBER, LOW);    // turn the LED off by making the voltage LOW
  // // delay(100);                       // wait for a second
  //
  // while(!isBusIdle());
  //
  // tinbusWrite(0x10);
  // delayMicroseconds(500);
  // tinbusWrite(0x31);
  // delayMicroseconds(500);
  // tinbusWrite(0x72);
  // delayMicroseconds(500);
  // tinbusWrite(0xF3);
  // // tinbusWrite(data++);
  // // tinbusWrite(data++);
}
