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
  pinMode(PIN_LED_AMBER, OUTPUT);
  tinbusBegin();
  // Serial.begin(19200);
}

void loop() {

  digitalWrite(PIN_LED_AMBER, digitalRead(PIN_LED_AMBER) == 0);
  delayMicroseconds(30000);

  while(!isBusIdle());
  tinbusWrite(0x10);
  tinbusWrite(0x31);
  tinbusWrite(0x72);
  tinbusWrite(0xF3);

  while(!isBusIdle());
  tinbusWrite(0xF3);
  tinbusWrite(0x72);
  tinbusWrite(0x31);
  tinbusWrite(0x10);
}
