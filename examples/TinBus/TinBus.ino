#include "TinBus.h"

#define kRxInterruptPin (2)
#define kBaudRate (1200L)
#define kSerialPort (Serial1)
void rxCallback(uint8_t *data, uint8_t length);

TinBus tinBus(kSerialPort, kBaudRate, kRxInterruptPin, rxCallback);

void hexDump(char *tag, uint8_t *buffer, int size) {
  int i = 0;
  Serial.print(tag);
  Serial.print(" : ");
  while (i < size) {
    Serial.print(buffer[i] >> 4, HEX);
    Serial.print(buffer[i++] & 0x0F, HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  tinBus.begin();
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  static unsigned long secondsTimer = 0;

  tinBus.update();

  unsigned long m = millis();
  if(m - secondsTimer > 1000L){  // send message once per second
    secondsTimer = m;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // toggle led

    uint8_t data[] = "Hello World!";
    uint8_t length = strlen(data);
    tinBus.write(data, length, MEDIUM_PRIORITY);
    hexDump("Send : ", data, length);
  }
}

void rxCallback(uint8_t *data, uint8_t length){
  hexDump("Recieve : ", data, length);
}
