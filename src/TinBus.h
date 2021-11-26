#ifndef TINBUS_H
#define TINBUS_H

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TINBUS_NO_DATA (-1)
#define TINBUS_BIT_OVERUN (-2)
#define TINBUS_BUFFER_FULL (-3)
#define TINBUS_END_OF_FRAME (-4)
#define TINBUS_MISSED_FRAME (-5)

void tinbusBegin(void);
int16_t tinbusRead(void);
int16_t tinbusWrite(uint8_t txByte);
bool isBusIdle(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // TINBUS_H
