/*
  ____  _       _       ____   ____       _             
 |  _ \(_) __ _(_)     |  _ \ / ___|     | | ___  _   _ 
 | | | | |/ _` | |_____| |_) | |      _  | |/ _ \| | | |
 | |_| | | (_| | |_____|  __/| |___  | |_| | (_) | |_| |
 |____/|_|\__, |_|     |_|    \____|  \___/ \___/ \__, |
          |___/                                   |___/ 

 A Control Freak sidekick project by Danjovic

 Basic release - 14 april 2020

*/


//    _ _ _                 _
//   | (_) |__ _ _ __ _ _ _(_)___ ___
//   | | | '_ \ '_/ _` | '_| / -_|_-<
//   |_|_|_.__/_| \__,_|_| |_\___/__/
//
#include "DigiJoystick.h"
#include <avr/sleep.h>
#include <avr/power.h>

//                 _      _    _
//   __ ____ _ _ _(_)__ _| |__| |___ ___
//   \ V / _` | '_| / _` | '_ \ / -_|_-<
//    \_/\__,_|_| |_\__,_|_.__/_\___/__/
//
uint8_t aX, aY, fireButtons;
uint8_t interval;
volatile uint8_t i = 0; // used in empty counting loop, must be volatile


//    ___      _
//   / __| ___| |_ _  _ _ __
//   \__ \/ -_)  _| || | '_ \
//   |___/\___|\__|\_,_| .__/
//                     |_|
void setup() {
  pinMode(5,INPUT_PULLUP); // Button 1
  pinMode(1,INPUT_PULLUP); // Button 2  
  pinMode(0,INPUT);        // Y axis
  pinMode(2,INPUT);        // X axis
  interval = 0;
}


//    __  __      _        _
//   |  \/  |__ _(_)_ _   | |   ___  ___ _ __
//   | |\/| / _` | | ' \  | |__/ _ \/ _ \ '_ \
//   |_|  |_\__,_|_|_||_| |____\___/\___/ .__/
//                                      |_|
void loop() {

  sleep_enable();              // Prepare CPU to sleep 
  power_timer1_disable();      // turn housekeeping off
  sleep_cpu();                 // 
  sleep_disable();             // 

  if (++interval > 3) {        // run once in row of three
    interval = 0;
    for (i = 0; i < 150; i++); // 100us delay
    doNewSample();            
    populateValues();
    DigiJoystick.update(); //
  }
}


//     __              _   _
//    / _|_  _ _ _  __| |_(_)___ _ _  ___
//   |  _| || | ' \/ _|  _| / _ \ ' \(_-<
//   |_|  \_,_|_||_\__|\__|_\___/_||_/__/
//


//
inline void populateValues() {
  // populate values
  DigiJoystick.setX((byte) aX);
  DigiJoystick.setY((byte) aY);
  DigiJoystick.setXROT((byte) 0x80);
  DigiJoystick.setYROT((byte) 0x80);
  DigiJoystick.setZROT((byte) 0x80);
  DigiJoystick.setSLIDER((byte) 0x80);
  DigiJoystick.setButtons((uint8_t)fireButtons, (uint8_t)0);
}

//
void doNewSample(void) {
  uint8_t sample,countVar;

  // Release capacitors to charge
  DDRB &= ~( (1<<0) | (1<<2) ) ;  

  // now count time it takes for each input to flip HIGH 
  aY = 0; aX = 0, countVar=255;
  do {
    sample = ~PINB;
    aY += sample & (1 << 0); //
    sample >>= 2;
    aX += sample & (1 << 0);
  } while (--countVar);

  // reset external capacitors
  DDRB  |=  ( (1<<0) | (1<<2) ) ;  // pin as outputs
  PORTB &= ~( (1<<0) | (1<<2) ) ;  // write zero to output

  // Update the buttons
  fireButtons = 0;
  if (~PINB & (1<<1)) fireButtons|=1;
  if (~PINB & (1<<5)) fireButtons|=2;
}
