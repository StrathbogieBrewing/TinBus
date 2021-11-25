#include "TinBus.h"

#if defined(__AVR_ATmega8__)

#define CPU_MHZ (8)

// this should be the period between clock pulses - 100 us
#define CLOCK_TIME (100 * CPU_MHZ)

#define TX_PULSE_TIME (CLOCK_TIME / 2)
#define TX_BYTE_TIME (2 * CLOCK_TIME)
// #define TX_FRAME_TIME (4 * CLOCK_TIME)

#define RX_IGNORE_TIME (CLOCK_TIME / 4)
#define RX_BYTE_TIMEOUT ((3 * CLOCK_TIME / 2) - RX_IGNORE_TIME)
#define RX_FRAME_TIMEOUT (3 * CLOCK_TIME / 2)
#define RX_IDLE_TIMEOUT (5 * CLOCK_TIME)

#define RX_THRESHOLD (3 * CLOCK_TIME / 4)

#define TIMER1_MAX (0xFFFF)

#define RX_IDLE (0)
#define RX_IGNORE (1)
#define RX_SAMPLING (2)
#define RX_BYTE_READY (3)
#define RX_FRAME_READY (4)

#define TX_SENDING (10)
#define TX_ABORT (11)
#define TX_BYTE_BREAK (12)
#define TX_FRAME_BREAK (13)

#define RX_EMPTY (-1)
#define RX_ERROR (-2)
#define RX_COUNT_ERROR (-3)
#define RX_BUFFER_ERROR (-4)

void sendPulse(void);
void timer1CompA(void);

static volatile uint8_t state = RX_IDLE;
static volatile uint8_t pulseCounter = 0;

static volatile uint8_t rxShiftReg = 0;
static volatile int16_t rxData = RX_EMPTY;

static volatile uint8_t txData = 0;

#define TINBUS_BUFFER_SIZE (64)
#define TINBUS_BUFFER_MASK (0x3F)

volatile uint8_t rx_buffer_head = 0;
volatile uint8_t rx_buffer_tail = 0;
volatile uint8_t tx_buffer_head = 0;
volatile uint8_t tx_buffer_tail = 0;

uint8_t rx_buffer[TINBUS_BUFFER_SIZE];
uint8_t tx_buffer[TINBUS_BUFFER_SIZE];

void tinbusBegin(void) {
  // set up timer 1 for ATMEGA8
  TCCR1A = 0;
  TCCR1B = (1 << CS10); // no prescaling - 8 MHz

  // set up analog comparator
  ACSR |= (1 << ACIC); // Analog Comparator Input Capture Enable

  // start receiving
  state = RX_IDLE;
  pulseCounter = 0;
  rxData = RX_EMPTY;
  TIFR |= (1 << ICF1);    // clear input capture interrupt flag
  TIMSK |= (1 << TICIE1); // enable interrupt capture interrupt
}



bool isBusIdle(void){
  return(state == RX_IDLE);
}


bool tinbusWrite(uint8_t txByte){
  // PORTB |= (1 << 1);
  // PORTB &= ~(1 << 1);

  uint8_t i = (tx_buffer_head + 1) & TINBUS_BUFFER_MASK;
  if(i == tx_buffer_tail){
    return false;         // buffer full
  }

  tx_buffer[tx_buffer_head] = txByte;  // append data to buffer
  tx_buffer_head = i;

  if(state == RX_IDLE){
    PORTB |= (1 << 1);
    PORTB &= ~(1 << 1);
    PORTB |= (1 << 1);
    PORTB &= ~(1 << 1);


    state = TX_BYTE_BREAK;
    // TIFR |= (1 << OCF1A);    // clear timer interrupt flag
    OCR1A = TCNT1 + 10;
    TIMSK |= (1 << OCIE1A);  // enable timer compare interrupt
    // noInterrupts();
    // timer1CompA();
    // interrupts();
  }

  return true;
}

int16_t tinbusRead(void){
  noInterrupts();
  if(rx_buffer_head == rx_buffer_tail){  // buffer empty
    interrupts();
    return RX_EMPTY;
  }

  uint8_t rxByte = rx_buffer[rx_buffer_tail++];
  tx_buffer_tail &= TINBUS_BUFFER_MASK;
  interrupts();
  return (uint16_t)rxByte;

  // noInterrupts();
  // int16_t retval = rxData;
  // // if(state == RX_IDLE){
  //   rxData = RX_EMPTY;
  //   interrupts();
  //   return retval;
  // }
  // interrupts();
  // return RX_EMPTY;
}

void timer1CompA(void){
  if(state == TX_SENDING){
    if(TIFR & (1 << ICF1)){
      // state = TX_ABORT;  // abort transmission if we have seen any other pulses
      // PORTB |= (1 << 1);
      // PORTB &= ~(1 << 1);
    }
    if (++pulseCounter & 0x01) {
      sendPulse();  // send clock pulse
      if (pulseCounter & 0x10) {  // is send byte complete
        // TIMSK &= ~(1 << OCIE1A);    // disable timer interrupt
        // state = RX_IDLE;
        // state = RX_FRAME_READY;

        state = TX_BYTE_BREAK;  // byte break if more tx data to send
        OCR1A += TX_BYTE_TIME;

        // if(tx_buffer_head == tx_buffer_tail){
        //   state = TX_FRAME_BREAK;  // frame break if no more tx data
        //   OCR1A += TX_FRAME_TIME;
        // } else {
        //   state = TX_BYTE_BREAK;  // byte break if more tx data to send
        //   OCR1A += TX_BYTE_TIME;
        // }

        // pulseCounter = 0;

        // TIFR |= (1 << ICF1);    // clear input capture interrupt flag
        // TIMSK |= (1 << TICIE1); // enable input capture interrupt

      } else {
        OCR1A += TX_PULSE_TIME;  // always at least one period
      }
    } else {
      if ((txData & 0x80) == 0) {
        sendPulse(); // send data pulse if bit is zero
      }
      txData <<= 1;  // move to next bit
      OCR1A += TX_PULSE_TIME;  // always at least one period
    }
    TIFR |= (1 << ICF1);    // clear input capture interrupt flag

  } else if(state == RX_IGNORE){
    state = RX_SAMPLING;
    OCR1A += RX_BYTE_TIMEOUT;
    TIFR |= (1 << ICF1);    // clear input capture interrupt flag
    TIMSK |= (1 << TICIE1); // enable input capture interrupt
  } else if(state == RX_SAMPLING){
    state = RX_BYTE_READY;  // rx byte timeout - end of byte
    OCR1A += RX_FRAME_TIMEOUT;

    if(pulseCounter == 17){  // complete byte received
      uint8_t i = (rx_buffer_head + 1) & TINBUS_BUFFER_MASK;
      if(i == rx_buffer_tail){  // rx buffer is full
        rxData = RX_BUFFER_ERROR;
      } else {  // append rx data to buffer
        rx_buffer[rx_buffer_head] = rxShiftReg;
        rx_buffer_head = i;
      }

      PORTB |= (1 << 1);
      PORTB &= ~(1 << 1);
    } else {
      rxData = RX_COUNT_ERROR;
    }
    pulseCounter = 0;

  } else if(state == RX_BYTE_READY){  // rx frame timeout - end of frame
    // state = RX_FRAME_READY;
    // OCR1A = RX_IDLE_TIMEOUT;

    state = RX_IDLE;          // return to idle - allows for transmitting again
    TIMSK &= ~(1 << OCIE1A);  // disable timer interrupt

    PORTB |= (1 << 1);
    PORTB &= ~(1 << 1);

  } else if(state == RX_FRAME_READY){
    state = RX_IDLE;          // return to idle - allows for transmitting again
    TIMSK &= ~(1 << OCIE1A);  // disable timer interrupt


  } else if(state == TX_BYTE_BREAK){
    if(tx_buffer_head == tx_buffer_tail){
      state = TX_FRAME_BREAK;  // frame break if no more tx data
      OCR1A += TX_BYTE_TIME;
    } else {
      // state = TX_BYTE_BREAK;  // byte break if more tx data to send
      // OCR1A += TX_BYTE_TIME;
      state = TX_SENDING;
      sendPulse();              // send first clock pulse
      pulseCounter = 1;
      txData = tx_buffer[tx_buffer_tail++];
      tx_buffer_tail &= TINBUS_BUFFER_MASK;
      OCR1A += TX_PULSE_TIME;
      TIMSK &= ~(1 << TICIE1);  // disable input capture interrupt
      TIFR |= (1 << ICF1);      // clear input capture interrupt flag
    }



    // state = TX_SENDING;
    // sendPulse();              // send first clock pulse
    // pulseCounter = 1;
    // txData = tx_buffer[tx_buffer_tail++];
    // tx_buffer_tail &= TINBUS_BUFFER_MASK;
    // OCR1A += TX_PULSE_TIME;
    // TIMSK &= ~(1 << TICIE1);  // disable input capture interrupt
    // TIFR |= (1 << ICF1);      // clear input capture interrupt flag
  } else if(state == TX_FRAME_BREAK){
    state = RX_IDLE;          // return to idle - allows for transmitting again
    TIMSK &= ~(1 << OCIE1A);  // disable timer interrupt
  }
  // else if(state == TX_START){
  //
  //   state = RX_IDLE;          // return to idle - allows for transmitting again
  //   TIMSK &= ~(1 << OCIE1A);  // disable timer interrupt
  // }
}


ISR(TIMER1_COMPA_vect) {
  timer1CompA();
}

ISR(TIMER1_CAPT_vect) {  // for receiving pulses
  uint16_t icr1 = ICR1; // reset and sychronise timer to this pulse
  TCNT1 = TCNT1 - icr1;
  if(pulseCounter++){   // ignore the first pulse
    if(icr1 > RX_THRESHOLD){  // must have received a clock pulse
      pulseCounter++;  // increment counter for the missing data pulse
      rxShiftReg = (rxShiftReg << 1) | 0x01;  // and shift in the one
    } else {
      if((pulseCounter & 0x01) == 0){  // must have received a data pulse
        rxShiftReg = (rxShiftReg << 1) | 0x00;  // shift in the zero
      }
    }
  }
  state = RX_IGNORE;
  OCR1A = RX_IGNORE_TIME;  // set ignore period
  TIMSK |= (1 << OCIE1A);  // enable timer interrupt
  TIMSK &= ~(1 << TICIE1); // disable input capture interrupt
  TIFR |= (1 << OCF1A);    // clear timer interrupt flag
}

void sendPulse(void) {
  // 250 ns periods with 8 mhz clock, 7 step sequence 0, +, +, 0, -, -, 0
  // 120 deg conduction eliminates 3rd harmonic and reduces thd
  uint8_t ddrd = (DDRD & ~((1 << 6) | (1 << 7)));
  DDRD = ddrd | (1 << 6) | (1 << 7);
  PORTD |= (1 << 7);
  _NOP();
  _NOP();
  PORTD |= (1 << 6);
  PORTD &= ~(1 << 7);
  _NOP();
  _NOP();
  PORTD &= ~(1 << 6);
  _NOP();
  DDRD = ddrd;
  PORTD &= ~(1 << 7);
  PORTD &= ~(1 << 6);
}

// if(tx_buffer_head == tx_buffer_tail){
//   state = RX_IDLE;          // return to idle - allows for transmitting again
//   TIMSK &= ~(1 << OCIE1A);  // disable timer interrupt
// } else {
//   state = TX_BYTE_BREAK;  // byte break if more tx data to send
//   OCR1A += TX_BYTE_TIME;
// }

// ISR(TIMER1_COMPA_vect) {
//   if (++pulseCounter & 0x01) {
//     // clock pulse
//     sendPulse();
//     if (pulseCounter & 0x10) {
//       // tx complete
//       TIMSK &= ~(1 << OCIE1A); // disable compare interrupt
//     }
//   } else {
//     // data pulse
//     if (txData & 0x80) {
//       // send data pulse
//       sendPulse();
//     }
//     txData <<= 1;
//   }
// }

  // PORTB |= (1 << 1);
  // PORTB &= ~(1 << 1);

  // if (++pulseCounter & 0x01) {
  //   // pre data rx phase
  //   if (TIFR & (1 << ICF1)) {
  //     // should not have received a pulse yet - ABORT
  //     return;
  //   }
  //   if (pulseCounter & 0x10) {
  //     // rx complete
  //     rxCallback(rxShiftReg);
  //     rxShiftReg = 0;
  //     pulseCounter = 0;
  //   } else {
  //     OCR1B = RX_LISTEN_UNTIL;
  //   }
  //
  // } else {
  //   // post data rx phase
  //   rxShiftReg <<= 1;
  //   if (TIFR & (1 << ICF1)) {
  //     rxShiftReg &= 0x01;
  //   }
  //
  //   TIMSK &= ~(1 << OCIE1B); // disable timer interrupt
  //
  //   // enable comparator interrupt and wait for clock pulse
  //   TIFR |= (1 << ICF1);    // clear input capture interrupt flag
  //   TIMSK |= (1 << TICIE1); // enable interrupt capture interrupt
  // }


// ISR(TIMER1_COMPA_vect) {
//   if (++pulseCounter & 0x01) {
//     // clock pulse
//     sendPulse();
//     if (pulseCounter & 0x10) {
//       // tx complete
//       TIMSK &= ~(1 << OCIE1A); // disable compare interrupt
//     }
//   } else {
//     // data pulse
//     if (txData & 0x80) {
//       // send data pulse
//       sendPulse();
//     }
//     txData <<= 1;
//   }
// }

// void begin(){
//   // set up timer 1 for ATMEGA8
//   TCCR1A = 0;
//   TCCR1B = (1 << CS10) | (1 << WGM12); // ctc with no prescaling - 8 MHz
//
//   // set up analog comparator
//   ACSR |= (1 << ACIC); // Analog Comparator Input Capture Enable
//
//   // start receiving
//   TIFR |= (1 << ICF1);    // clear input capture interrupt flag
//   TIMSK |= (1 << TICIE1); // enable interrupt capture interrupt
//
//     // TIMSK |= (1 << TICIE1);
//     // TIMSK |= (1 << OCIE1A);
//     // TIMSK |= (1 << OCIE1B);
//     // TIMSK |= (1 << TOIE1);
// }

// bool transmitByte(uint8_t byte) {
//
//   noInterrupts();
//   if(state == RX_IDLE){
//     PORTB |= (1 << 1);
//     PORTB &= ~(1 << 1);
//
//     TIMSK &= ~(1 << TICIE1); // disable input capture interrupt
//     OCR1A = TCNT1 + TX_PULSE_TIME;
//     sendPulse();            // send first clock pulse
//     pulseCounter = 1;
//     state = TX_SENDING;
//     txData = byte;
//     TIFR |= (1 << OCF1A);    // clear timer interrupt flag
//     TIMSK |= (1 << OCIE1A);  // enable timer compare interrupt
//     TIFR |= (1 << ICF1);    // clear input capture interrupt flag
//     interrupts();
//     return true;
//   }
//   interrupts();
//   return false;
// }

// bool transmitEnable(uint8_t byte) {
//   if(state == RX_IDLE){  // need to start transmitting
//     sendPulse();              // send first clock pulse
//     OCR1A = TCNT1 + TX_PULSE_TIME;
//     pulseCounter = 1;
//     state = TX_SENDING;
//     txData = txByte;
//
//     TIFR |= (1 << OCF1A);    // clear timer interrupt flag
//     TIMSK |= (1 << OCIE1A);  // enable timer compare interrupt
//     TIFR |= (1 << ICF1);    // clear input capture interrupt flag
//     interrupts();
//     return true;
//   }
// }

#endif
