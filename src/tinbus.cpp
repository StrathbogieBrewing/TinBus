#include "tinbus.h"

#define CPU_MHZ (8)

#define CLOCK_PERIOD (125 * CPU_MHZ)

#define TX_DATA_TIME (CLOCK_PERIOD / 2)

#define RX_IGNORE_TIME (CLOCK_PERIOD / 4)
#define RX_BYTE_TIMEOUT (3 * CLOCK_PERIOD / 2)
#define RX_FRAME_TIMEOUT (3 * CLOCK_PERIOD)
#define RX_IDLE_TIMEOUT (5 * CLOCK_PERIOD)

#define RX_THRESHOLD (3 * CLOCK_PERIOD / 4)

#define RX_SAMPLE_TIME (CLOCK_PERIOD / 4)
#define TIMER1_MAX (0xFFFF)

#define RX_IDLE (0)
#define RX_IGNORE (1)
#define RX_SAMPLE (2)
#define RX_BYTE_READY (3)
#define RX_FRAME_READY (4)

#define TX_SEND (10)

#define RX_EMPTY (-1)
#define RX_ERROR (-2)
#define RX_COUNT_ERROR (-3)

void sendPulse(void);

static volatile uint8_t state = RX_IDLE;
static volatile uint8_t pulseCounter = 0;

static volatile uint8_t rxShiftRegister = 0;
static volatile int16_t rxData = RX_EMPTY;

static volatile uint8_t txData = 0;


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

bool transmitByte(uint8_t byte) {

  noInterrupts();
  if(state == RX_IDLE){
    PORTB |= (1 << 1);
    PORTB &= ~(1 << 1);

    TIMSK &= ~(1 << TICIE1); // disable input capture interrupt
    OCR1A = TCNT1 + TX_DATA_TIME;
    sendPulse();            // send first clock pulse
    TIFR |= (1 << ICF1);    // clear input capture interrupt flag
    pulseCounter = 1;
    state = TX_SEND;
    txData = byte;
    TIFR |= (1 << OCF1A);    // clear timer interrupt flag
    TIMSK |= (1 << OCIE1A);  // enable timer compare interrupt
    interrupts();
    return true;
  }
  interrupts();
  return false;
}




ISR(TIMER1_COMPA_vect) {

  if(state == TX_SEND){
    OCR1A += TX_DATA_TIME;

    if(TIFR & (1 << ICF1)){
      // abort transmission if we have seen any other pulses
      PORTB |= (1 << 1);
      PORTB &= ~(1 << 1);
    }

    if (++pulseCounter & 0x01) {
      sendPulse();  // send clock pulse
      if (pulseCounter & 0x10) {  // is tx complete
        // TIMSK &= ~(1 << OCIE1A);    // disable timer interrupt
        // state = RX_IDLE;
        state = RX_FRAME_READY;
        OCR1A += TX_DATA_TIME * 3;  // need 2 clock cycles between bytes

        TIFR |= (1 << ICF1);    // clear input capture interrupt flag
        TIMSK |= (1 << TICIE1); // enable input capture interrupt
        pulseCounter = 0;
      }
    } else {
      if (txData & 0x80) {
        sendPulse(); // send data pulse
      }
      txData <<= 1;  // move to next bit
    }

    TIFR |= (1 << ICF1);    // clear input capture interrupt flag

  } else if(state == RX_IGNORE){
    state = RX_SAMPLE;
    OCR1A = RX_BYTE_TIMEOUT;
    TIFR |= (1 << ICF1);    // clear input capture interrupt flag
    TIMSK |= (1 << TICIE1); // enable input capture interrupt
  } else if(state == RX_SAMPLE){
    state = RX_BYTE_READY;  // rx byte timeout - end of byte
    OCR1A = RX_FRAME_TIMEOUT;

    // rxData = RX_EMPTY;
    // if(rxData == RX_EMPTY){
      if(pulseCounter == 17){
        rxData = (uint16_t)rxShiftRegister;
        PORTB |= (1 << 1);
        PORTB &= ~(1 << 1);
      } else {
        rxData = RX_COUNT_ERROR;
      }
      pulseCounter = 0;
    // }

  } else if(state == RX_BYTE_READY){
    state = RX_FRAME_READY;  // rx frame timeout - end of frame
    OCR1A = RX_IDLE_TIMEOUT;


  } else if(state == RX_FRAME_READY){
    state = RX_IDLE;          // return to idle - allows for transmitting again
    TIMSK &= ~(1 << OCIE1A);  // disable timer interrupt

    PORTB |= (1 << 1);
    PORTB &= ~(1 << 1);
  }
}

ISR(TIMER1_CAPT_vect) {
  uint16_t icr1 = ICR1;       // reset and sychronise timer to this pulse
  TCNT1 = TCNT1 - icr1;

  if(pulseCounter++){  // ignore the first pulse
    if(icr1 > RX_THRESHOLD){
      // must have received a clock pulse
      pulseCounter++;  // increment counter for the missing data pulse
      rxShiftRegister = (rxShiftRegister << 1) | 0x00;  // and shift in the zero
    } else {
      if((pulseCounter & 0x01) == 0){
        // must have received a data pulse
        rxShiftRegister = (rxShiftRegister << 1) | 0x01;
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
  uint8_t ddrd = (DDRD & ~((1 << 6) | (1 << 7)));
  DDRD = ddrd | (1 << 6) | (1 << 7);
  _NOP();
  _NOP();
  PORTD |= (1 << 7);
  _NOP();
  _NOP();
  PORTD |= (1 << 6);
  _NOP();
  _NOP();
  PORTD &= ~(1 << 7);
  _NOP();
  _NOP();
  PORTD &= ~(1 << 6);
  _NOP();
  _NOP();
  _NOP();
  DDRD = ddrd;
  PORTD &= ~(1 << 7);
  PORTD &= ~(1 << 6);
}

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
  //     rxCallback(rxShiftRegister);
  //     rxShiftRegister = 0;
  //     pulseCounter = 0;
  //   } else {
  //     OCR1B = RX_LISTEN_UNTIL;
  //   }
  //
  // } else {
  //   // post data rx phase
  //   rxShiftRegister <<= 1;
  //   if (TIFR & (1 << ICF1)) {
  //     rxShiftRegister &= 0x01;
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
