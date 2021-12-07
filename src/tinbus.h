#ifndef TINBUS_H
#define TINBUS_H

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
  TINBUS_RX_NO_DATA = 0,
  TINBUS_RX_BIT_OVERUN,
  TINBUS_RX_BUFFER_FULL,
  TINBUS_RX_FRAME_RECEIVED,
  TINBUS_RX_FRAME_OVERUN,
  TINBUS_RX_DROPPED_FRAME,

  TINBUS_TX_IDLE,
  TINBUS_TX_ACCEPTED,
  TINBUS_TX_ABORTED,
  TINBUS_TX_BUSY,
  TINBUS_TX_SUCCESS,
};

// maximum frame size must be power of 2 and no more than 64 bytes
#define TINBUS_MAX_FRAME_SIZE (32)

typedef struct {
  uint8_t size;
  uint8_t data[TINBUS_MAX_FRAME_SIZE];
} tinbus_frame_t;

void tinbus_init(void);
uint8_t tinbus_read(tinbus_frame_t *frame);
uint8_t tinbus_write(tinbus_frame_t *frame);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // TINBUS_H
