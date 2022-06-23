#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "gcr.h"

#define GCR_START (0x01)
#define GCR_STOP (0x10)
#define GCR_CODE_MASK (0x1f)
#define GCR_INVALID (0Xff)
#define GCR_CODE_SIZE (5)

// clang-format off

/* GCR with codes in ascending order, 4 bits > 5 bits */
static const unsigned char gcr_encode[16] = {
    0x09, 0x0a, 0x0b, 0x0d,
    0x0e, 0x0f, 0x12, 0x13,
    0x15, 0x16, 0x17, 0x19,
    0x1a, 0x1b, 0x1d, 0x1e,
};

/* GCR with codes in ascending order, 5 bits > 4 bits (0xff => invalid) */
static const unsigned char gcr_decode[32] = {
    0xff, 0xff, 0xff, 0xff, /* 0x00 - 0x03 invalid */
    0xff, 0xff, 0xff, 0xff, /* 0x04 - 0x07 invalid */
    0xff, 0x00, 0x01, 0x02, /* 0x08 invalid        */
    0xff, 0x03, 0x04, 0x05, /* 0x0C invalid        */

    0xff, 0xff, 0x06, 0x07, /* 0x10 - 0x11 invalid */
    0xff, 0x08, 0x09, 0x0a, /* 0x14 invalid        */
    0xff, 0x0b, 0x0c, 0x0d, /* 0x18 invalid        */
    0xff, 0x0e, 0x0f, 0xff, /* 0x1C, 0x1F invalid  */
};

// clang-format on

char *gcr_dump(char *buf, char code) {
  char i = 0x10;
  code &= GCR_CODE_MASK;
  while (i) {
    if (i & code) {
      *buf++ = '1';
    } else {
      *buf++ = '0';
    }
    i >>= 1;
  }
  *buf = '\0';
  return buf;
}

struct gcr_ctx {
  uint16_t shift_reg;
  uint8_t bits;
  uint8_t data;
  enum gcr_state state;
};

enum gcr_state gcr_in(struct gcr_ctx *ctx, bool bit) {
  /* Update state of receiver */
  if ((ctx->state == GCR_DATA_READY) || (ctx->state == GCR_START_OF_FRAME)) {
    ctx->state = GCR_RECEIVING;
    ctx->bits = 0; /* Reset receive buffer and wait for next byte*/
  }

  if ((ctx->state == GCR_END_OF_FRAME) || (ctx->state == GCR_ERROR)) {
    ctx->state = GCR_IDLE;
    ctx->bits = 0; /* Reset receive buffer and wait for start*/
  }

  /* Limit bit count to 16 to avoid rollover after many '0's */
  if ((ctx->bits & 0x10) != 0) {
    ctx->bits++;
  }
  /* Shift new data into register */
  ctx->shift_reg <<= 1;
  ctx->shift_reg |= bit;

  /* Mask code words bits */
  uint8_t code = ctx->shift_reg & GCR_CODE_MASK;

  /* Check for start code */
  if ((code == GCR_START) && (ctx->bits >= 5)) {
    ctx->state = GCR_START_OF_FRAME;
    return ctx->state;
  }

  /* Check for new valid code */
  if (ctx->state == GCR_RECEIVING) {
    if ((ctx->bits == 5) || (ctx->bits == 10)) {
      /* Check for stop code first as its an invalid data code */
      if (code == GCR_STOP) {
        ctx->state = GCR_END_OF_FRAME;
        return ctx->state;
      }

      /* Decode the received code */
      uint8_t nibble = gcr_decode[code];
      if (nibble == GCR_INVALID) {
        ctx->state = GCR_ERROR;
        return ctx->state;
      }

      if (ctx->bits == 5) {
        /* Store most significant nibble */
        ctx->data = nibble << 4;
        return ctx->state;
      } else {
        /* Store least significant nibble */
        ctx->data |= nibble;
        ctx->state = GCR_DATA_READY;
        return ctx->state;
      }
    }
  }

  /* Fall through in IDLE or RECEIVING states */
  return ctx->state;
}

void gcr_rx(char *buf, char *msg) {
  struct gcr_ctx ctx = {0};
  while (*buf) {
    bool bit = ((*buf++) == '1');
    // printf("%d", bit);
    enum gcr_state state = gcr_in(&ctx, bit);
    switch (state) {
    case GCR_DATA_READY:
      // printf("decoded %c\n", ctx.data);
      *msg++ = ctx.data;
      break;
    case GCR_START_OF_FRAME:
      // printf("start of frame\n");
      break;
    case GCR_END_OF_FRAME:
      // printf("end of frame\n");
      break;
    case GCR_ERROR:
      printf("error \n");
      break;
    case GCR_RECEIVING:
      // printf("receiving \n");
      break;
    case GCR_IDLE:
      // printf("idle \n");
      break;
    }
  }
  *msg = '\0';
}

void gcr_tx(char *buf, char *msg) {
  buf = gcr_dump(buf, GCR_START);
  while (*msg) {
    char code = gcr_encode[(*msg >> 4) & 0x0F];
    buf = gcr_dump(buf, code);
    code = gcr_encode[*msg & 0x0F];
    buf = gcr_dump(buf, code);
    msg++;
  }
  buf = gcr_dump(buf, GCR_STOP);
}

int main() {
  char buf[1024] = {0};
  char message[] = "Hello World";

  gcr_tx(buf, message);

  printf("%s\n", buf);
  printf("bits %lu, bytes %lu\n", strlen(buf), strlen(message));

  char rx_msg[1024] = {0};
  gcr_rx(buf, rx_msg);
  printf("%s\n", rx_msg);
}
