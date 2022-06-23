#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "tty.h"

static int fd = -1;
static struct termios original_termios;

int tty_open(char *port) {
  // open the serial port
  int rc = -1;
  struct termios termios;
  speed_t baud = B9600;

  fd = open(port, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    fprintf(stderr, "TTY Device \"%s\" could not be opened\n", port);
    return -1;
  }

  if (!isatty(fd)) {
    fprintf(stderr, "TTY Device \"%s\" is not a tty\n", port);
    return -1;
  }
  // initialize terminal modes
  rc = tcgetattr(fd, &termios);
  if (rc < 0) {
    fprintf(stderr, "TTY Device tcgetattr() failed\n");
    return -1;
  }
  original_termios = termios;
  termios.c_iflag = IGNBRK;
  termios.c_oflag = 0;
  termios.c_lflag = 0;
  termios.c_cflag = (CS8 | CREAD | CLOCAL);
  termios.c_cc[VMIN] = 0;
  termios.c_cc[VTIME] = 5;
  if (cfsetospeed(&termios, baud) < 0) {
    fprintf(stderr, "TTY Device cfsetospeed() failed\n");
    return -1;
  }
  if (cfsetispeed(&termios, baud) < 0) {
    fprintf(stderr, "TTY Device cfsetispeed() failed\n");
    return -1;
  }
  rc = tcsetattr(fd, TCSANOW, &termios);
  if (rc < 0) {
    fprintf(stderr, "TTY Device tcsetattr() failed\n");
    return -1;
  }
  return 0;
}

int tty_read(packet_t *packet) {
  if (fd < 0) {
    return -1;
  }
  return packet->size = read(fd, packet->buffer, tty_kBufferSize);
}

int tty_write(packet_t *packet) {
  return write(fd, packet->buffer, packet->size);
}

void tty_close(void) {
  if (fd != -1) {
    close(fd);
    fd = -1;
  }
}
