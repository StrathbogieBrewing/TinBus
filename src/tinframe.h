#ifndef TINFRAME_H
#define TINFRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#define tinframe_kOK (0)
#define tinframe_kCRCError (1)
#define tinframe_kFrameError (2)

extern unsigned char tinframe_priority[];

#define tinframe_kMaxDataBytes (16)
typedef struct {
  unsigned char priority;
  unsigned char dataLength;
  unsigned char data[tinframe_kMaxDataBytes];
  unsigned char crc;
} tinframe_t;
#define tinframe_kFrameSize (sizeof(tinframe_t))
#define tinframe_kFrameOverhead (tinframe_kFrameSize - tinframe_kMaxDataBytes)

void tinframe_prepareFrame(tinframe_t *frame);
char tinframe_checkFrame(const tinframe_t *frame);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // TINFRAME_H
