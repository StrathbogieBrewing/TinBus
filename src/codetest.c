#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// #include <time.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include "tty.h"

int8_t tinbus_decode(uint8_t code);

// 0b11111111,
// 0b11111011,
// 0b11101111,
// 0b11101101,
// 0b11101011,
// 0b10111111,
// 0b10111101,
// 0b10110101,
// 0b10101111,
// 0b10101101,
// 0b10101011,
// 0b01111011,
// 0b01101111,
// 0b01101101,
// 0b01101011,
// 0b01011011,
// 0b01010101,

#define TINBUS_SOF (0b11111111)

// clang-format off
/* Codes compatible with UART in 8N1 mode */
static const uint8_t tinbus_code_table[16] = {
  // 0b10111111,
  // 0b11101111,
  // 0b01101111,
  // 0b10101111,
  // 0b11111011,
  // 0b01111011,
  // 0b01011011,  
  // 0b11101011,
  // 0b01101011,
  // 0b10101011,
  // 0b10111101,
  // 0b11101101,
  // 0b01101101,
  // 0b10101101,
  // 0b10110101,  
  // 0b01010101

  0b11111011,
  0b11101111,
  0b11101101,
  0b11101011,
  0b10111111,
  0b10111101,
  0b10110101,
  0b10101111,
  0b10101101,
  0b10101011,
  0b01111011,
  0b01101111,
  0b01101101,
  0b01101011,
  0b01011011,
  0b01010101,

};
// clang-format on

uint8_t tinbus_encode(uint8_t code) { return tinbus_code_table[code & 0x0F]; }

char *bin_dump(char *buf, uint16_t code) {
  uint16_t i = 0x8000;
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

struct test_result {
  uint8_t candidate;
  uint8_t ones_run_length;
  uint8_t zeroes_run_length;
  uint8_t zeroes_count;
};

void test_code(uint16_t code, struct test_result *result) {
  uint8_t ones = 0;
  uint8_t zeroes = 0;

  result->ones_run_length = 0;
  result->zeroes_run_length = 0;
  result->zeroes_count = 0;

  for (uint8_t i = 0; i < 13; i++) {
    bool bit = (1 << i) & code;
    if (bit) {
      if (zeroes > result->zeroes_run_length) {
        result->zeroes_run_length = zeroes;
      }
      zeroes = 0;
      ones++;
    } else {
      if (ones > result->ones_run_length) {
        result->ones_run_length = ones;
      }
      result->zeroes_count++;
      zeroes++;
      ones = 0;
    }
  }
}

#define kProgramName (0)
#define kSerialDevice (1)

static volatile int keepRunning = 1;
void intHandler(int dummy) { keepRunning = 0; }

int main(int argc, char *argv[]) {
  struct test_result candidates[256];
  uint8_t candidate_index = 0;
  char buf[64];

  for (int16_t i = 255; i >= 0; i--) {
    uint16_t vector = ((i | 0xFD00) << 2) | 0x01;
    bin_dump(buf, vector);
    struct test_result result;
    result.candidate = i;
    test_code(vector, &result);
    if ((result.zeroes_run_length == 1) && (result.zeroes_count < 6) &&
        ((result.ones_run_length == 1) || (result.ones_run_length == 2) ||
         (result.ones_run_length == 4) || (result.ones_run_length == 6) ||
         (result.ones_run_length == 9))) {
      candidates[candidate_index++] = result;
    }
  }

  for (uint8_t i = 0; i < candidate_index; i++) {
    bin_dump(buf, candidates[i].candidate);
    printf("%3d %s total zeroes %3d, ones run length %3d\n", i, buf,
           candidates[i].zeroes_count, candidates[i].ones_run_length);
  }

  // return 0;

  signal(SIGINT, intHandler);
  if (argc < 2) {
    fprintf(stdout, "Usage: %s <serial device>\n", argv[kProgramName]);
    return EXIT_FAILURE;
  }
  if (tty_open(argv[kSerialDevice]) == -1) {
    return EXIT_FAILURE;
  }
  uint8_t x = 0;
  packet_t txPacket;
  while (keepRunning) {
    txPacket.buffer[0] = TINBUS_SOF;
    txPacket.buffer[1] = tinbus_encode((x >> 4) & 0x0F);
    txPacket.buffer[2] = tinbus_encode(x & 0x0F);
    txPacket.buffer[3] = TINBUS_SOF;
    txPacket.size = 4;
    usleep(1000000);
    tty_write(&txPacket);
    x--;

    printf("%3d encodes to %3d and then decodes to %3d\n", (x & 0x0F),
           tinbus_encode(x & 0x0F), tinbus_decode(tinbus_encode(x & 0x0F)));
  }
  tty_close();

  return 0;
}

int8_t tinbus_decode(uint8_t code) {
  /* Implement fast look up using a partial binary search */
  if (code > tinbus_code_table[8]) {
    if (code > tinbus_code_table[4]) {
      if (code == tinbus_code_table[0])
        return 0;
      if (code == tinbus_code_table[1])
        return 1;
      if (code == tinbus_code_table[2])
        return 2;
      if (code == tinbus_code_table[3])
        return 3;
    } else {
      if (code == tinbus_code_table[4])
        return 4;
      if (code == tinbus_code_table[5])
        return 5;
      if (code == tinbus_code_table[6])
        return 6;
      if (code == tinbus_code_table[7])
        return 7;
    }
  } else {
    if (code > tinbus_code_table[12]) {
      if (code == tinbus_code_table[8])
        return 8;
      if (code == tinbus_code_table[9])
        return 9;
      if (code == tinbus_code_table[10])
        return 10;
      if (code == tinbus_code_table[11])
        return 11;
    } else {
      if (code == tinbus_code_table[12])
        return 12;
      if (code == tinbus_code_table[13])
        return 13;
      if (code == tinbus_code_table[14])
        return 14;
      if (code == tinbus_code_table[15])
        return 15;
    }
  }
  return -1;
}