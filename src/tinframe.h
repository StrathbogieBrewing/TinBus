#ifndef TINFRAME_H
#define TINFRAME_H

#ifdef __cplusplus
extern "C" {
#endif

enum {
  tinframe_kOK = 0,
  tinframe_kCRCError,
  tinframe_kFrameError,
};

#define tinframe_kDataSize (10)
#define tinframe_kStart (0x7E)  // same as PPP frame, why not?

typedef struct {
  unsigned char start;
  unsigned char data[tinframe_kDataSize];
  unsigned char crc;
} tinframe_t;

#define tinframe_kFrameSize (sizeof(tinframe_t))

void tinframe_prepareFrame(tinframe_t *frame);
int tinframe_checkFrame(const tinframe_t *frame);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // TINFRAME_H
