#include "tinbus.h"

#if defined(__AVR_ATmega8__)

static inline uint16_t TIMER_SYNC(void) __attribute__((always_inline));
static inline uint16_t TIMER_SYNC(void) {
  uint16_t x = ICR1 - OCR1B;
  OCR1A = ICR1;
  OCR1B = ICR1; // dodgy! used to store capture value
  return x;
}

static inline void TIMER_WAIT(uint16_t t) __attribute__((always_inline));
static inline void TIMER_WAIT(uint16_t t) {
  OCR1A += t;             // set wait period
  TIMSK |= (1 << OCIE1A); // enable timer interrupt
  TIFR |= (1 << OCF1A);   // clear timer interrupt flag
}

static inline void TIMER_INIT(void) __attribute__((always_inline));
static inline void TIMER_INIT(void) {
  TCCR1A = 0;           // set up timer 1 for ATMEGA8
  TCCR1B = (1 << CS10); // no prescaling - 8 MHz
}

static inline void TIMER_DISABLE(void) __attribute__((always_inline));
static inline void TIMER_DISABLE(void) {
  TIMSK &= ~(1 << OCIE1A); // disable timer interrupt
}

static inline void RXINT_CLEAR(void) __attribute__((always_inline));
static inline void RXINT_CLEAR(void) {
  TIFR |= (1 << ICF1); // clear input capture interrupt flag
}

static inline void RXINT_DISABLE(void) __attribute__((always_inline));
static inline void RXINT_DISABLE(void) {
  TIMSK &= ~(1 << TICIE1); // disable input capture interrupt
}

static inline void RXINT_ENABLE(void) __attribute__((always_inline));
static inline void RXINT_ENABLE(void) {
  RXINT_CLEAR();
  TIMSK |= (1 << TICIE1); // enable input capture interrupt
}

static inline bool RXINT_CHANGED(void) __attribute__((always_inline));
static inline bool RXINT_CHANGED(void) { return TIFR & (1 << ICF1); }

static inline void RXINT_INIT(void) __attribute__((always_inline));
static inline void RXINT_INIT(void) {
  ACSR |= (1 << ACIC); // analog Comparator Input Capture Enable
}

#define CPU_MHZ (8)

// this should be the period between clock pulses - 400 us
#define CLOCK_TIME (400 * CPU_MHZ)

#define TX_PULSE_TIME (CLOCK_TIME / 2)
#define TX_BYTE_TIME (2 * CLOCK_TIME)

#define RX_IGNORE_TIME (CLOCK_TIME / 4)
#define RX_BYTE_TIMEOUT ((3 * CLOCK_TIME / 2) - RX_IGNORE_TIME)
#define RX_FRAME_TIMEOUT (3 * CLOCK_TIME / 2)
#define RX_THRESHOLD (3 * CLOCK_TIME / 4)

#define RX_IDLE (0)
#define RX_IGNORE (1)
#define RX_SAMPLING (2)
#define RX_BYTE_READY (3)
#define RX_FRAME_READY (4)

#define TX_SENDING (10)
#define TX_ABORT (11)
#define TX_BYTE_BREAK (12)
#define TX_FRAME_BREAK (13)

#if ((TINBUS_MAX_FRAME_SIZE & (TINBUS_MAX_FRAME_SIZE - 1)) ||                  \
     (TINBUS_MAX_FRAME_SIZE > 64) || (TINBUS_MAX_FRAME_SIZE == 0))
#error TINBUS_MAX_FRAME_SIZE Must be a power of 2 and not more than 64 bytes
#endif

#define TINBUS_TX_BUFFER_SIZE (TINBUS_MAX_FRAME_SIZE)
#define TINBUS_RX_BUFFER_SIZE (TINBUS_MAX_FRAME_SIZE * 2)
#define TINBUS_RX_BUFFER_MASK (TINBUS_RX_BUFFER_SIZE - 1)
#define TINBUS_NO_FRAME_IN_BUFFER (TINBUS_RX_BUFFER_SIZE)

static volatile uint8_t state = RX_IDLE;
static volatile uint8_t pulseCounter = 0;

static volatile uint8_t txShiftReg = 0;
static volatile uint8_t tx_buffer_index = 0;
static volatile uint8_t tx_buffer_frame_size = TINBUS_NO_FRAME_IN_BUFFER;
static volatile uint8_t txMessage = TINBUS_TX_IDLE;

static volatile uint8_t rxShiftReg = 0;
static volatile uint8_t rx_buffer_head = 0;
static volatile uint8_t rx_buffer_tail = 0;
static volatile uint8_t rx_buffer_frame_end = TINBUS_NO_FRAME_IN_BUFFER;
static volatile uint8_t rxMessage = TINBUS_RX_NO_DATA;

static uint8_t rx_buffer[TINBUS_RX_BUFFER_SIZE];
static uint8_t tx_buffer[TINBUS_TX_BUFFER_SIZE];

static void sendPulse(void);

void tinbus_init(void) {
  TIMER_INIT();
  RXINT_INIT();
  RXINT_ENABLE();
}

uint8_t tinbus_write(tinbus_frame_t *frame) {
  // PORTB |= (1 << 1);
  // PORTB &= ~(1 << 1);
  noInterrupts();
  uint8_t msg = txMessage;
  txMessage = TINBUS_TX_IDLE;
  uint8_t frame_size = tx_buffer_frame_size;
  interrupts();
  if (msg != TINBUS_TX_IDLE) {
    return msg;
  }
  if (frame_size != TINBUS_NO_FRAME_IN_BUFFER) {
    return TINBUS_TX_BUSY;
  }
  memcpy(tx_buffer, frame->data, frame->size); // copy frame to buffer
  noInterrupts();
  tx_buffer_frame_size = frame->size;
  tx_buffer_index = 0;

  if (state == RX_IDLE) {
    state = TX_BYTE_BREAK;
    // TIMER_WAIT(TX_BYTE_TIME);
    OCR1A = TCNT1 + 10;
    TIMSK |= (1 << OCIE1A); // enable timer compare interrupt
  }
  interrupts();
  return TINBUS_TX_ACCEPTED;
}

uint8_t tinbus_read(tinbus_frame_t *frame) {
  noInterrupts();
  uint8_t msg = rxMessage;
  rxMessage = TINBUS_RX_NO_DATA;
  uint8_t frame_end = rx_buffer_frame_end;
  interrupts();
  if (msg != TINBUS_RX_NO_DATA) { // report error message
    return msg;
  }
  if (frame_end == TINBUS_NO_FRAME_IN_BUFFER) {
    return TINBUS_RX_NO_DATA;
  }

  noInterrupts(); // recover received frame from rx buffer
  uint8_t buffer_tail = rx_buffer_tail;
  rx_buffer_tail = frame_end;
  rx_buffer_frame_end = TINBUS_NO_FRAME_IN_BUFFER;
  interrupts();
  uint8_t *data = frame->data;
  uint8_t size = 0;
  while (buffer_tail != frame_end) {
    *data++ = rx_buffer[buffer_tail++];
    buffer_tail &= TINBUS_RX_BUFFER_MASK;
    if (++size >= TINBUS_MAX_FRAME_SIZE) {
      return TINBUS_RX_FRAME_OVERUN;
    }
  }
  frame->size = size;
  return TINBUS_RX_FRAME_RECEIVED;
}

ISR(TIMER1_COMPA_vect) {
  if (state == TX_SENDING) {
    if (RXINT_CHANGED()) {
      txMessage = TINBUS_TX_ABORTED; // abort tx if we have seen another pulse
      state = TX_FRAME_BREAK;        // force frame break
      TIMER_WAIT(TX_BYTE_TIME);
    } else {
      if (++pulseCounter & 0x01) {
        sendPulse();               // send clock pulse
        if (pulseCounter & 0x10) { // is send byte complete
          state = TX_BYTE_BREAK;   // byte break if more tx data to send
          TIMER_WAIT(TX_BYTE_TIME);
        } else {
          TIMER_WAIT(TX_PULSE_TIME); // always at least one period
        }
      } else {
        if ((txShiftReg & 0x80) == 0) {
          sendPulse(); // send data pulse if bit is zero
        }
        txShiftReg <<= 1;          // move to next bit
        TIMER_WAIT(TX_PULSE_TIME); // always at least one period
      }
    }
    RXINT_CLEAR(); // clear rx input changed flag
  } else if (state == RX_IGNORE) {
    state = RX_SAMPLING;
    TIMER_WAIT(RX_BYTE_TIMEOUT);
    RXINT_ENABLE();
  } else if (state == RX_SAMPLING) {
    state = RX_BYTE_READY; // rx byte timeout - end of byte
    TIMER_WAIT(RX_FRAME_TIMEOUT);
    if (pulseCounter == 17) { // complete byte received
      uint8_t i = (rx_buffer_head + 1) & TINBUS_RX_BUFFER_MASK;
      if (i == rx_buffer_tail) { // rx buffer is full
        rxMessage = TINBUS_RX_BUFFER_FULL;
      } else { // append rx data to buffer
        rx_buffer[rx_buffer_head] = rxShiftReg;
        rx_buffer_head = i;
      }
    } else {
      rxMessage = TINBUS_RX_BIT_OVERUN;
    }
    pulseCounter = 0;
  } else if (state == RX_BYTE_READY) { // rx frame timeout - end of frame
    state = RX_IDLE; // return to idle - allows for transmitting again
    TIMER_DISABLE();
    if (rx_buffer_frame_end !=
        TINBUS_NO_FRAME_IN_BUFFER) { // discard previous frame
      rx_buffer_tail = rx_buffer_frame_end;
      rxMessage = TINBUS_RX_FRAME_DROPPED;
    }
    rx_buffer_frame_end = rx_buffer_head;
  } else if (state == TX_BYTE_BREAK) {
    if (tx_buffer_index == tx_buffer_frame_size) {
      state = TX_FRAME_BREAK; // frame break if no more tx data
      TIMER_WAIT(TX_BYTE_TIME);
    } else {
      state = TX_SENDING;
      sendPulse(); // send first clock pulse
      pulseCounter = 1;
      txShiftReg = tx_buffer[tx_buffer_index++];
      TIMER_WAIT(TX_PULSE_TIME);
      RXINT_DISABLE();
      RXINT_CLEAR();
    }
  } else if (state == TX_FRAME_BREAK) {
    state = RX_IDLE; // return to idle - allows for transmitting again
    tx_buffer_frame_size = TINBUS_NO_FRAME_IN_BUFFER;
    TIMER_DISABLE();
  }
}

ISR(TIMER1_CAPT_vect) { // for receiving pulses
  uint16_t icr1 = TIMER_SYNC();
  if (pulseCounter++) {        // ignore the first pulse
    if (icr1 > RX_THRESHOLD) { // must have received a clock pulse
      pulseCounter++;          // increment counter for the missing data pulse
      rxShiftReg = (rxShiftReg << 1) | 0x01; // and shift in the one
    } else {
      if ((pulseCounter & 0x01) == 0) { // must have received a data pulse
        rxShiftReg = (rxShiftReg << 1) | 0x00; // shift in the zero
      }
    }
  }
  state = RX_IGNORE;
  TIMER_WAIT(RX_IGNORE_TIME);
  RXINT_DISABLE();
  RXINT_CLEAR();
}

void sendPulse(void) {
  DDRD = (1 << 6);
  PORTD |= (1 << 6);
  delayMicroseconds(2);
  _NOP();
  _NOP();
  _NOP();
  _NOP();
  _NOP();
  _NOP();
  _NOP();
  _NOP();
  _NOP();
  _NOP();
  _NOP();
  _NOP();
  _NOP();
  _NOP();
  // _NOP();
  // _NOP();
  PORTD &= ~(1 << 6);
}

// void sendPulse(void) {
//   // always called from ISR - wont be interrupted
//   // 250 ns periods with 8 mhz clock, 7 step sequence 0, +, +, 0, -, -, 0
//   // 120 deg conduction eliminates 3rd harmonic and reduces thd
//   uint8_t ddrd = (DDRD & ~((1 << 6) | (1 << 7)));
//   DDRD = ddrd | (1 << 6) | (1 << 7);
//   PORTD |= (1 << 7);
//   _NOP();
//   _NOP();
//   PORTD |= (1 << 6);
//   PORTD &= ~(1 << 7);
//   _NOP();
//   _NOP();
//   PORTD &= ~(1 << 6);
//   _NOP();
//   DDRD = ddrd;
//   PORTD &= ~(1 << 7);
//   PORTD &= ~(1 << 6);
// }

#endif
