#ifndef TLC5941_H
#define TLC5941_H
#endif

//TLC5941 pin connected to Bean
#define XLAT   A1
#define MODE   0
#define BLANK  1
#define GSCLK  2
#define SOUT   4
#define SCLK   5

//TLC5941 Configuration Data

// Amount of clock ticks for half a PWM pulse with no pre-scaling.
// At 80 cycles we will get a PWM frequency of 50kHz and 
// max update rate (BLANK pulses) of roughly 12Hz (8000000/160/4096 = ~12).
#define CLOCK_CYCLES     80

//Number of PWM pulses in a full PWM cycle at which time we send a BLANK pulse.
//See the TLC5941 datasheet for this.
#define PWM_PULSES     4096

//Number of full PWM cycles between XLAT and triggering next IO refresh loop.
//If this is set to 1 the IO refresh loop will happen ~12 times a second.
//Careful if you are sharing the same serial bus with other components as 
//the TLC5941 could latch invalid data when this is run in the ISR. 
//To protect from this set the g_dontlatch global to 1 while using other serial
//components on the same bus.
#define PWM_CYCLES        3

class TLC5941Class {
  private:
  
  static int pwm_pulse_count; //Keeps track of the number of PWM pulses that passed
  static int pwm_cycle_count; //Keeps track of the number of full PWM cycles that passed
  static volatile uint8_t dontlatch; //flag 

  //Buffer for TLC5941 DC values
  static uint8_t dcvalues[12];
  
  //Buffer for TLC5941 GS values
  static uint8_t gsvalues[24];

  public:

  //Call in setup to initialize the TLC5941
  static void init ();

  //The ISR that is called for each PWM Pulse
  static void pwmPulseISR();
  
  //Call this to prevent invalid data from being latched into the TLC5941 gray scale
  //registers. This is always set to 0 after an XLAT pulse since the shift registers
  //will be filled with status bits after an XLAT. You must set it back
  //to 1 after shifting in new data or XLAT will never get pulsed again.
  static void dontLatch () { dontlatch = 1; }

  //Call this to allow data to be latched into the TLC5941 gray scale registers.
  static void canLatch () { dontlatch = 0; }
  
  //Set the dcvalue for a specific outpin between 0-63
  static void setDC (uint8_t outpin, uint8_t dcvalue);

  //Set the gsvalue for a specific outpin between 0-4095
  static void setOutput (uint8_t outpin, int gsvalue);

  //Shift in data and set dontlatch to 0
  static void shiftInAll ();
};

extern TLC5941Class TLC5941;

