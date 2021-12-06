#ifndef SLTIN_H
#define SLTIN_H

#include <stdint.h>

#include "tinbus.h"

#ifdef __cplusplus
extern "C" {
#endif



typedef void (*slwrite_f)(uint8_t byte);
//
// typedef struct {
//   uint8_t  size;
//   uint8_t  data[sltin_kBufferSize];
// } sltin_t;
//
// uint8_t sltin_sendFrame(sltin_t *frame, slwrite_f slwrite);
uint8_t sltin_sendFrame(tinbus_frame_t *frame, slwrite_f slwrite);

// uint8_t sltin_enframe(sltin_t *frame);


// uint8_t sltin_deframe(sltin_t *frame, char inByte);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* SLTIN_H_ */
