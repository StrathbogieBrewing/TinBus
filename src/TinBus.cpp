#include "TinBus.h"

#define kRXDataReady (tinframe_kFrameSize + 1)
#define kRXDone (tinframe_kFrameSize + 2)

#define kTXRequest (tinframe_kFrameSize + 1)
#define kTXIdle (tinframe_kFrameSize + 2)

TinBus::TinBus(HardwareSerial &serial, unsigned long baud, unsigned char interruptPin, TinBus_rxCallback callback)
    : serialPort{serial}, baudRate{baud},rxInterruptPin{interruptPin}, rxCallback{callback} {}

volatile unsigned long TinBus::rxActiveMicros = 0;

void TinBus::externalInterrupt(void) { rxActiveMicros = micros(); }

void TinBus::begin() {
  serialPort.begin(baudRate);
  pinMode(rxInterruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(rxInterruptPin), externalInterrupt,
                  CHANGE);
  rxIndex = kRXDone;
  txIndex = kTXIdle;
}

int TinBus::update() {
  noInterrupts();
  unsigned long lastActivity = micros() - rxActiveMicros;
  interrupts();
  if (lastActivity > TinBus_kInterFrameMicros) {
    rxIndex = 0;
    while (serialPort.available() > 0) {
      serialPort.read();
    }
    unsigned int txPriority = TinBus_kInterFrameMicros;
    unsigned char firstDataByte = txFrame.data[0];  // priority based on byte 0
    while(firstDataByte){
      txPriority += TinBus_kBitPeriodMicros;
      firstDataByte <<= 1;
    }
    if ((lastActivity > txPriority) && (digitalRead(rxInterruptPin) == HIGH)){
      if (txIndex == kTXRequest) {
        txIndex = 0;
        unsigned char txData = ((char *)&txFrame)[txIndex];
        serialPort.write(txData);
        noInterrupts();
        rxActiveMicros = micros(); // immediately update rxActiveMicros
        interrupts();
        return TinBus_kWriteBusy;
      } else {
        if(txIndex != kTXIdle){
          txIndex = kTXIdle;
          return TinBus_kWriteTimeout;
        }
        txIndex = kTXIdle;
      }
    }
  }
  if (txIndex < tinframe_kFrameSize) {
    unsigned char txData = ((char *)&txFrame)[txIndex];
    if (serialPort.available() > 0) {
      unsigned char rxData = serialPort.read();
      if (rxData == txData) {
        ++txIndex;
        if (txIndex < tinframe_kFrameSize) {
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
    unsigned char rxData = serialPort.read();
    if (rxIndex < tinframe_kFrameSize) {
      ((char *)&rxFrame)[rxIndex++] = rxData;
    } else {
      return TinBus_kReadOverunError;
    }
    if (rxIndex == tinframe_kFrameSize) {
      rxIndex = kRXDone;
      if (tinframe_checkFrame(&rxFrame) == tinframe_kOK) {
        rxCallback(&rxFrame);
        return TinBus_kOK;
      } else {
        return TinBus_kReadCRCError;
      }
    }
  }
  return TinBus_kOK;
}

int TinBus::write(tinframe_t *frame) {
  if (txIndex != kTXIdle) {
    return TinBus_kWriteBusy;
  }
  tinframe_prepareFrame(frame);
  memcpy(&txFrame, frame, tinframe_kFrameSize);
  txIndex = kTXRequest;
  return TinBus_kOK;
}
