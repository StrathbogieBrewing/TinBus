#include "Arduino.h"

void tinbusBegin(void);
bool transmitByte(uint8_t byte);
int16_t receiveByte(void);
bool tinbusWrite(uint8_t txByte);
bool isBusIdle(void);
