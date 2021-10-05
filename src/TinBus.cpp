#include "TinBus.h"

#define kRXDone (tinframe_kFrameSize + 1)
#define kTXRequest (tinframe_kFrameSize + 1)
#define kTXIdle (tinframe_kFrameSize + 2)

TinBus::TinBus(HardwareSerial &serial, unsigned long baud,
               uint8_t interruptPin, TinBus_rxCallback callback)
    : serialPort{serial}, baudRate{baud}, rxInterruptPin{interruptPin},
      rxCallback{callback} {}

volatile unsigned long TinBus::rxActiveMicros = 0;

void TinBus::externalInterrupt(void) { rxActiveMicros = micros(); }

void TinBus::begin() {
  serialPort.begin(baudRate);
  bitPeriodMicros = 1000000UL / baudRate;
  interFrameMicros = bitPeriodMicros * 15L;
  pinMode(rxInterruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(rxInterruptPin), externalInterrupt,
                  CHANGE);
  rxIndex = kRXDone;
  txIndex = kTXIdle;
}

char TinBus::update() {
  noInterrupts();
  unsigned long lastActivity = micros() - rxActiveMicros;
  interrupts();
  if (lastActivity > interFrameMicros) {
    rxIndex = 0;                          // reset rx buffer
    while (serialPort.available() > 0) {
      serialPort.read();                  // and purge rx fifo
    }
    if (txIndex == kTXRequest) {
      if ((lastActivity > txHoldOff) &&
          (digitalRead(rxInterruptPin) == HIGH)) {
        txIndex = 0;
        uint8_t txData = ((char *)&txFrame)[txIndex];
        serialPort.write(txData);         // start tx
        noInterrupts();
        rxActiveMicros = micros();        // immediately update rxActiveMicros
        interrupts();
        return TinBus_kWriteBusy;
      }
    }
    if (lastActivity > (interFrameMicros * 2)){
      if (txIndex != kTXIdle) {
        txIndex = kTXIdle;
        return TinBus_kWriteTimeout;
      }
    }
  }
  if (txIndex < tinframe_kFrameSize) {
    uint8_t txData = ((char *)&txFrame)[txIndex];
    if (serialPort.available() > 0) {
      uint8_t rxData = serialPort.read();
      if (rxData == txData) {
        ++txIndex;
        if (txIndex < txFrame.dataLength + tinframe_kFrameOverhead) {
          txData = ((char *)&txFrame)[txIndex];
          serialPort.write(txData);
          return TinBus_kWriteBusy;
        } else {
          txIndex = kTXIdle;
          return TinBus_kWriteComplete;
        }
      } else {
        txIndex = kTXIdle;
        return TinBus_kWriteCollision;
      }
    }
    return TinBus_kWriteBusy;
  }

  if (serialPort.available() > 0) {
    uint8_t rxData = serialPort.read();
    if (rxIndex < tinframe_kFrameSize) {
      ((char *)&rxFrame)[rxIndex++] = rxData;
    } else {
      return TinBus_kReadOverrun;
    }
    if (rxIndex == (rxFrame.dataLength + tinframe_kFrameOverhead)) {
      rxIndex = kRXDone;
      if (tinframe_checkFrame(&rxFrame) == tinframe_kOK) {
        rxCallback(rxFrame.data, rxFrame.dataLength);
        return TinBus_kOK;
      } else {
        return TinBus_kReadCRCError;
      }
    }
  }
  return TinBus_kOK;
}

char TinBus::write(uint8_t *data, uint8_t length, uint8_t priority) {
  if (txIndex != kTXIdle) {
    return TinBus_kWriteBusy;
  }
  priority &= 0x07;
  txFrame.priority = tinframe_priority[priority];
  txHoldOff = interFrameMicros + bitPeriodMicros * (uint32_t)priority;
  txFrame.dataLength = length;
  if(length > tinframe_kMaxDataBytes){
    return TinBus_kWriteOverrun;
  }
  memcpy(txFrame.data, data, length);
  tinframe_prepareFrame(&txFrame);
  txIndex = kTXRequest;
  return TinBus_kOK;
}
