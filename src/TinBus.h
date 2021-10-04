#ifndef TINBUS_H
#define TINBUS_H

#include "Arduino.h"
#include "TinBusError.h"
#include "tinframe.h"

typedef void (*TinBus_rxCallback)(unsigned char *data, unsigned char length);

class TinBus {
public:
  TinBus(HardwareSerial &serial, unsigned long baud, unsigned char interruptPin,
         TinBus_rxCallback callback);
  void begin();
  char update();
  char write(unsigned char *data, unsigned char length, unsigned char priority);

private:
  HardwareSerial &serialPort;
  unsigned char rxInterruptPin;
  TinBus_rxCallback rxCallback;
  unsigned long baudRate;
  unsigned int bitPeriodMicros;
  unsigned int interFrameMicros;

  tinframe_t txFrame;
  unsigned char txIndex;
  unsigned char txHoldOff;
  tinframe_t rxFrame;
  unsigned char rxIndex;

  static void externalInterrupt(void);
  static volatile unsigned long rxActiveMicros;
};

#endif //  TINBUS_H
