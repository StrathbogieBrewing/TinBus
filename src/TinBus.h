#ifndef TINBUS_H
#define TINBUS_H

#include "Arduino.h"
#include "TinBusError.h"
#include "tinframe.h"

#define TinBus_kBitPeriodMicros (1000000UL / TinBus_kBaud)
#define TinBus_kInterFrameMicros (TinBus_kBitPeriodMicros * 15)

typedef void (*TinBus_rxCallback)(tinframe_t *frame);

class TinBus {
public:
  TinBus(HardwareSerial &serial, unsigned long baud, unsigned char interruptPin,
         TinBus_rxCallback callback);
  void begin();
  int update();
  int write(tinframe_t *frame);

private:
  HardwareSerial &serialPort;
  unsigned char rxInterruptPin;
  TinBus_rxCallback rxCallback;
  unsigned long baudRate;
  unsigned int bitPeriodMicros;
  unsigned int interFrameMicros;

  tinframe_t txFrame;
  unsigned char txIndex;
  tinframe_t rxFrame;
  unsigned char rxIndex;

  static void externalInterrupt(void);
  static volatile unsigned long rxActiveMicros;
};

#endif //  TINBUS_H
