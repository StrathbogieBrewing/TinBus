#ifndef TINBUS_H
#define TINBUS_H

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

void tinbusBegin(void);
bool transmitByte(uint8_t byte);
int16_t tinbusRead(void);
bool tinbusWrite(uint8_t txByte);
bool isBusIdle(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // TINBUS_H
