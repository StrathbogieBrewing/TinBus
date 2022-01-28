#include <stdio.h>
#include <stdint.h>

#define MAX_STRLEN (8)
#define ARRAY_SIZE (15)

// weights of the 8-bit values
static uint8_t small_table[256] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4,
    2, 3, 3, 4, 3, 4, 4, 5, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 1, 2, 2, 3, 2, 3, 3, 4,
    2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6,
    4, 5, 5, 6, 5, 6, 6, 7, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5,
    3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6,
    4, 5, 5, 6, 5, 6, 6, 7, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8};

uint8_t hammingWeight(uint16_t x) {
  return small_table[x & 0xFF] + small_table[(x >> 8) & 0xFF];
}

uint8_t hammingDistance(uint16_t x, uint16_t y) { return hammingWeight(x ^ y); }

// uint16_t findCode(uint16_t codes[], uint8_t base, uint8_t distance){
//   for (uint16_t i = 0; i < 256; i++) {
//
//     uint16_t test = base | (i << 8);
//
//     if(hammingDistance(code, test) < distance){
//       codes[i]
//     }
//     if()
//   }
// }

int main() {

  // int count = 0;
  uint16_t codes[256] = {0};

  // for (int i = 0; i < 256; i++) {
  //   codes[i] = i;
  //   // printf("%d\t%d\n", i, hammingDistance(i, 255));
  // }

  for (uint16_t i = 0; i < 256; i++) {  // all codes loop
    uint16_t code;


    for (uint16_t j = 0; j < i; j++) {  // completed codes loop
      for (uint16_t k = 0; k < 256; k++) {
        code = i | (k << 8);
        if(hammingDistance(code, codes[j]) < 2){
          codes[i]
        }
      }
    }
    codes[i] = code; //(i & 0xFF) | (k << 8);
  }
}
