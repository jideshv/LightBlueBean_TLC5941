#include "Arduino.h"
#include "tlc5941.h"

int TLC5941Class::pwm_pulse_count = 0;

int TLC5941Class::pwm_cycle_count = 0;

volatile uint8_t TLC5941Class::dontlatch = 0;

uint8_t TLC5941Class::dcvalues[12] = {255,255,255,255,255,255,255,255,255,255,255,255};

uint8_t TLC5941Class::gsvalues[24];

void TLC5941Class::init() {
  //Below code is custom to ATMEGA328P which is found on the Light Blue Bean
  unsigned char sreg;
  //Need to disable interrupts since the 16 bit write uses the 8 bit bus and a temporary register
  //Disabling the interrupts makes sure the TCNT1 and ICR1 write is atomic
  sreg = SREG;
  cli();
  //Stop clock for timer 1
  TCCR1B = 0;
  TCNT1 = 0;

  //Set TCCR1A based on which pin is set to PWM_CLOCK
  #if GSCLK == 1
  DDRB |= _BV(PORTB1);
  TCCR1A = _BV(COM1A1);
  #elif GSCLK == 2
  DDRB |= _BV(PORTB2);
  TCCR1A = _BV(COM1B1);
  #else
  #error "Only pins 1 and 2 are supported for GSCLK using this library since it users Timer1"
  #endif

  //Set ICR1 to TOP per PWM Mode 8 behavior
  ICR1 = CLOCK_CYCLES;

  //re-enable interrupts
  SREG = sreg;

  //Clock in Dot Correction values and save them.
  //TLC5941 does not have any EEPROM to save these so need to do this at start each time.
  
  //First set MODE of the TLC5941 to HIGH
  digitalWrite (MODE, HIGH);
  __asm__("nop\n\t"); //The minimum wait after setting MODE is ~10ns but just being safe

  //Clock in the DC values
  for (uint8_t i = 0; i < 12; i++) {
    shiftOut(SOUT, SCLK, MSBFIRST, dcvalues[i]);
  }

  //Latch DC values
  digitalWrite (XLAT, HIGH);
  __asm__("nop\n\t"); //The minimum wait after XLAT is ~10ns but just being safe
  digitalWrite (XLAT, LOW);
  
  //Set MODE of the TLC5941 to LOW to put it in GS mode
  digitalWrite (MODE, LOW);
  __asm__("nop\n\t"); //The minimum wait after setting MODE is ~10ns but just being safe

  //Clock in all zeros to the GS registers
  for (uint8_t i = 0; i < 24; i++) {
    gsvalues[i] = 0;
    shiftOut(SOUT, SCLK, MSBFIRST, 0);
  }

  //Latch the GS registers
  digitalWrite (XLAT, HIGH);
  __asm__("nop\n\t"); //The minimum wait after XLAT is ~10ns but just being safe
  digitalWrite (XLAT, LOW);

  //Per data sheet the first GS after a DC requires an additional SCLK after XLAT
  //Make sure SCLK is low first
  digitalWrite (SCLK, LOW);
  __asm__("nop\n\t");
  digitalWrite (SCLK, HIGH);
  __asm__("nop\n\t");
  digitalWrite (SCLK, LOW);
  
  //Setting BLANK to LOW to enable outputs
  digitalWrite (BLANK, LOW);

  //Set an interrupt on each PWM pulse so we can count them and reset at PWM_PULSES
  TIMSK1 = _BV(TOIE1);

  //Set Timer1 to PWM Mode 8 and no pre-scaling and start timer
  TCCR1B = _BV(WGM13) | _BV(CS10);
}

void TLC5941Class::setDC (uint8_t outpin, uint8_t dcvalue) {
  
}

void TLC5941Class::setOutput (uint8_t outpin, int gsvalue) {
  if ((outpin & 1) == 1) {
    //odd value means high portion is split
    gsvalue |= ~4095;
    uint8_t gsvaluel = gsvalue;
    uint8_t gsvalueh = gsvalue>>8;
  }
  else {
    //even value means low portion is split
  }
}

void TLC5941Class::shiftInAll () {
  //Clock in all gsvalues to the TLC5941 shift register
  //Assumes MODE remains LOW since nobody should be setting it
  //Caution using the same serial bus for another component prior
  //to the next interrupt driven XLAT pulse.
  for (uint8_t i = 0; i < 24; i++) {
    shiftOut(SOUT, SCLK, MSBFIRST, gsvalues[i]);
  }
}

//This is the interrupt service routine for the PWM pulse and cycle counter.
//It is written using bit manipulation on the port registers vs digitalWrite to keep things fast.
void TLC5941Class::pwmPulseISR ()
{
  if (pwm_pulse_count < PWM_PULSES) {
    pwm_pulse_count++;
    return;
  }
  else {
    //reset the PWM counter for the next cycle
    //replaces digitalWrite (BLANK, HIGH);
    uint8_t bbit = digitalPinToBitMask(BLANK);
    uint8_t bport = digitalPinToPort(BLANK);
    volatile uint8_t *bporeg = portOutputRegister(bport);
    *bporeg |= bbit;
    
    if (pwm_cycle_count < PWM_CYCLES) {
      pwm_pulse_count = 0;
      pwm_cycle_count++;
    }
    else {
      if (dontlatch != 1) {
        //replaces digitalWrite (XLAT, HIGH);
        uint8_t xbit = digitalPinToBitMask(XLAT);
        uint8_t xport = digitalPinToPort(XLAT);
        volatile uint8_t *xporeg = portOutputRegister(bport);
        *xporeg |= bbit;

        //Set an interrupt each time XLAT is triggered
        *digitalPinToPCMSK(XLAT) |= bit (digitalPinToPCMSKbit(XLAT));
        PCICR  |= bit (digitalPinToPCICRbit(XLAT));
        
        //replaces digitalWrite (XLAT, LOW);
        *xporeg &= ~bbit;

        //no longer latch till new data is shifted in
        dontlatch = 0;
      }
      pwm_pulse_count = 0;
      pwm_cycle_count = 0;
    }
    
    //replaces digitalWrite (PWM_BLANK, LOW);
    *bporeg &= ~bbit;
  }
}

TLC5941Class TLC5941;

ISR (TIMER1_OVF_vect) {
  TLC5941.pwmPulseISR();
}

