#ifndef TINBUS_H
#define TINBUS_H

#include "Arduino.h"
#include "TinBusError.h"
#include "tinframe.h"

#define HIGH_PRIORITY     (0x00)
#define MEDIUM_PRIORITY   (0x04)
#define LOW_PRIORIY       (0x07)

typedef void (*TinBus_rxCallback)(uint8_t *data, uint8_t length);

class TinBus {
public:
  TinBus(HardwareSerial &serial, unsigned long baud, uint8_t interruptPin,
         TinBus_rxCallback callback);
  void begin();
  char update();
  char write(uint8_t *data, uint8_t length, uint8_t priority);

private:
  HardwareSerial &serialPort;
  uint8_t rxInterruptPin;
  TinBus_rxCallback rxCallback;
  unsigned long baudRate;
  unsigned int bitPeriodMicros;
  unsigned int interFrameMicros;
  tinframe_t txFrame;
  uint8_t txIndex;
  unsigned long txHoldOff;
  tinframe_t rxFrame;
  uint8_t rxIndex;
  static void externalInterrupt(void);
  static volatile unsigned long rxActiveMicros;
};

#endif //  TINBUS_H
