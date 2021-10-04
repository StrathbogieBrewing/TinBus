#ifndef TINBUS_H
#define TINBUS_H

#include "Arduino.h"
#include "TinBusError.h"
#include "tinframe.h"

#define TinBus_kBitPeriodMicros (1000000UL / TinBus_kBaud)
#define TinBus_kInterFrameMicros (TinBus_kBitPeriodMicros * 15)

// first byte of tin frame data can be used to indicate frame prioirty
// this will adjust the bus silence period pre-transmit
// and avoid corruption of the first data byte
#define TinBus_kPriorityHighest  (0x00)
#define TinBus_kPriorityHighhigh (0x80)
#define TinBus_kPriorityHighmed  (0xC0)
#define TinBus_kPriorityMedhigh  (0xE0)
#define TinBus_kPriorityMedium   (0xF0)
#define TinBus_kPriorityMedlow   (0xF8)
#define TinBus_kPriorityLowmed   (0xFC)
#define TinBus_kPriorityLowlow   (0xFE)
#define TinBus_kPriorityLowest   (0xFF)

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

  tinframe_t txFrame;
  unsigned char txIndex;
  tinframe_t rxFrame;
  unsigned char rxIndex;

  static void externalInterrupt(void);
  static volatile unsigned long rxActiveMicros;
};

#endif //  TINBUS_H
