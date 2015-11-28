#include "tlc5941.h"

void setupPins () {
  pinMode (A0, OUTPUT);
  digitalWrite (A0, LOW);
  
  pinMode (XLAT, OUTPUT);
  digitalWrite (XLAT, LOW);
  
  pinMode (MODE, OUTPUT);
  digitalWrite (MODE, LOW);
  
  pinMode (BLANK, OUTPUT);
  digitalWrite (BLANK, HIGH);
  
  pinMode (GSCLK, OUTPUT);
  digitalWrite (GSCLK, LOW);
  
  pinMode (3, OUTPUT);
  digitalWrite (3, LOW);
  
  pinMode (SOUT, OUTPUT);
  digitalWrite (SOUT, LOW);
  
  pinMode (SCLK, OUTPUT);
  digitalWrite (SCLK, LOW);
}

//I will use this in the ISR for the XLAT change interrupt to tell
//the code in the loop function that we can shift in new values.
volatile uint8_t g_canupdate = 0;

//The TLC5941 update loop sets up the pin change interrupt on A1 (XLAT)
//before taking it low. This effectively mimics a software interrupt
//and must be used to update the data in the TLC5941 shift registers.
ISR(PCINT1_vect) {
  //unset interrupt since we explicity set it right before triggering it
  *digitalPinToPCMSK(XLAT) &= ~bit (digitalPinToPCMSKbit(XLAT));

  g_canupdate = 1;
}

void setup() {
  setupPins ();
  TLC5941.init();
}

void loop() {
  if (g_canupdate == 1) {
      TLC5941.setOutput(0,random(4095)); //r
      TLC5941.setOutput(1,random(4095)); //b
      TLC5941.setOutput(2,random(4095)); //g
      TLC5941.shiftInAll();
      g_canupdate = 0;
  }
}


