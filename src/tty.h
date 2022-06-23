#ifndef TTY_H_
#define TTY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#define tty_kBufferSize (256)

typedef struct {
  uint8_t size;
  uint8_t buffer[tty_kBufferSize];
} packet_t;

int tty_open(char *port);
int tty_read(packet_t *packet);
int tty_write(packet_t *packet);
void tty_close(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* TTY_H_ */
