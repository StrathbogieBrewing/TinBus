#ifndef GCR_H
#define GCR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum gcr_state
{
  GCR_IDLE = 0,
  GCR_START_OF_FRAME,
  GCR_RECEIVING,
  GCR_DATA_READY,
  GCR_END_OF_FRAME,
  GCR_ERROR,
};

struct gcr_ctx
{
  uint16_t shift_reg;
  uint8_t bits;
  uint8_t data;
  enum gcr_state state;
};

enum gcr_state gcr_in(struct gcr_ctx *ctx, bool bit);

enum gcr_state gcr_out(struct gcr_ctx *ctx, bool bit);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* GCR_H_ */
