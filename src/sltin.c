#include <string.h>

#include "sltin.h"

static uint8_t bin2hex(uint8_t bin) {
  bin &= 0x0F;
  if (bin < 10)
    return bin + '0';
  return bin + 'A' - 10;
}


uint8_t sltin_sendFrame(tinbus_frame_t *frame, slwrite_f slwrite) {
  uint8_t *input = frame->data;
  uint8_t byteCount = frame->size;
  // uint8_t buffer[sltin_kBufferSize];
  // uint8_t *output = buffer;
  uint8_t csum = 0x00;
  // uint8_t src = *input++;
  slwrite(':');
  // slwrite('I');
  slwrite(bin2hex(byteCount >> 4));
  slwrite(bin2hex(byteCount));
  // csum -= src;

  while (byteCount--) {
    uint8_t src = *input++;
    slwrite(bin2hex(src >> 4));
    slwrite(bin2hex(src));
    csum -= src;
  }
  csum += 0x55;
  slwrite(bin2hex(csum >> 4));
  slwrite(bin2hex(csum));
  // slwrite('\r');
  slwrite('\n');
  // slwrite('\0');

  // frame->size = output - buffer;
  // memcpy(frame->data, buffer, frame->size);

  return byteCount;
}
//
// static int8_t hex2bin(uint8_t hex) {
//   if (hex < '0')
//     return -1;
//   uint8_t val = hex - '0';
//   if (val > 9) {
//     val -= 7;
//     if ((val > 15) || (val < 10))
//       return -1;
//   }
//   return val;
// }
//
// uint8_t sltin_deframe(sltin_t *frame, char inByte) {
//   if (inByte == ':') {
//     frame->size = 0; // reset data buffer
//   }
//   if (frame->size < sltin_kBufferSize - 2) {
//     frame->data[frame->size++] = inByte;
//     if (inByte == '\n') {
//       frame->data[frame->size] = '\0';
//       uint8_t *input = frame->data;
//       uint8_t *output = frame->data;
//       uint8_t csum = 0x00;
//       while ((*input != '\n') && (*input != '\0')) {
//         uint8_t byte = hex2bin(*input++) << 4;
//         byte += hex2bin(*input++);
//         csum += byte;
//         *output++ = byte;
//       }
//       if (csum == 0x55) {
//         frame->size = output - frame->data - 1;
//         return frame->size; // size not including terminating null
//       }
//     }
//   }
//   return 0;
// }
