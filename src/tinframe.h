#ifndef TINFRAME_H
#define TINFRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#define tinframe_kOK (0)
#define tinframe_kCRCError (1)
#define tinframe_kFrameError (2)

// first byte of tin frame data is used to indicate frame start / prioirty
// this will adjust the required bus silence period pre-transmit
// the values below avoid corruption of the priority
#define tinframe_kPriorityHighest  (0x00)
#define tinframe_kPriorityHigh     (0x01)
#define tinframe_kPriorityHighmed  (0x02)
#define tinframe_kPriorityMedhigh  (0x03)
#define tinframe_kPriorityMedium   (0x04)
#define tinframe_kPriorityMedlow   (0x05)
#define tinframe_kPriorityLowmed   (0x06)
#define tinframe_kPriorityLow      (0x07)

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
