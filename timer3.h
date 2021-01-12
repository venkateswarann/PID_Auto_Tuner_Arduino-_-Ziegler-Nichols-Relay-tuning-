/*
   Timer 3 (set to 100hz) calculates the ground speed of each motor in m/s.
   Author: Venkateswaran Narayanan
   Ref: https://github.com/paulodowd/EMATM0054_20_21/blob/master/Labsheets/Supplementary/SL1_InterruptsAndTimers.ipynb
*/

#ifndef _TIMER_3_H_
#define _TIMER_3_H_

#include "Encoder.h"


volatile long count_r_old = 0;
volatile long count_l_old = 0;

volatile float speed_r;
volatile float speed_l;

const float pi = 3.1416;
const float wheel_dia = 0.07; // Diameter of Romi Wheel
const int cnt_per_rev = 1440;
const float d_per_count = (wheel_dia * pi) / (cnt_per_rev);
const float l = 0.145; // Wheel Seperation


volatile float const interval = 0.01;

// The ISR routine.
// The name TIMER3_COMPA_vect is a special flag to the
// compiler.  It automatically associates with Timer3 in
// CTC mode.
ISR( TIMER3_COMPA_vect ) {

  speed_r = (count_r - count_r_old) * d_per_count / interval;
  speed_l = (count_l - count_l_old) * d_per_count / interval;

  speed_r = int(speed_r * 1000)/1000.0;
  speed_l = int(speed_l * 1000)/1000.0;

//  count_r_old = count_e1;
//  count_l_old = count_e0;


  count_r_old = count_r;
  count_l_old = count_l;
}

// Routine to setupt timer3 to run
void setupTimer3() {

  // disable global interrupts
  cli();

  // Reset timer3 to a blank condition.
  // TCCR = Timer/Counter Control Register
  TCCR3A = 0;     // set entire TCCR3A register to 0
  TCCR3B = 0;     // set entire TCCR3B register to 0

  // First, turn on CTC mode.  Timer3 will count up
  // and create an interrupt on a match to a value.
  // See table 14.4 in manual, it is mode 4.
  TCCR3B = TCCR3B | (1 << WGM32);

  // For a cpu clock precaler of 256:
  // Shift a 1 up to bit CS32 (clock select, timer 3, bit 2)
  // Table 14.5 in manual.
  TCCR3B = TCCR3B | (1 << CS32);


  // set compare match register to desired timer count.
  // CPU Clock  = 16000000 (16mhz).
  // Prescaler  = 256
  // Timer freq = 16000000/256 = 62500
  // We can think of this as timer3 counting up to 62500 in 1 second.
  // compare match value = 62500 / 100 (we desire 100hz).
  OCR3A = 625;

  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei();
}

#endif
