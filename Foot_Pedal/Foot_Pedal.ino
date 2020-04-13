/*
   ___         _           _   ___             _
  / __|___ _ _| |_ _ _ ___| | | __| _ ___ __ _| |__
 | (__/ _ \ ' \  _| '_/ _ \ | | _| '_/ -_) _` | / /
  \___\___/_||_\__|_| \___/_| |_||_| \___\__,_|_\_\

  https://hackaday.io/project/170908-control-freak

  Danjovic 2020

  Foot Pedal Keyboard - 12/04/2020
*/



//    _ _ _                 _
//   | (_) |__ _ _ __ _ _ _(_)___ ___
//   | | | '_ \ '_/ _` | '_| / -_|_-<
//   |_|_|_.__/_| \__,_|_| |_\___/__/
//
#include "HID-Project.h"


//       _      __ _      _ _   _
//    __| |___ / _(_)_ _ (_) |_(_)___ _ _  ___
//   / _` / -_)  _| | ' \| |  _| / _ \ ' \(_-<
//   \__,_\___|_| |_|_||_|_|\__|_\___/_||_/__/
//

//                             AVR
#define joy1Button1 2    //  PD1/INT1
#define joy1Button2 3    //  PD0/INT0
#define joy2Button1 4    //  PD4
#define joy2Button2 6    //  PD7

#define joy1Xaxis   A3   //  PF4
#define joy1Yaxis   A2   //  PF5
#define joy2Xaxis   A1   //  PF6
#define joy2Yaxis   A0   //  PF7


#define holdCaps()    DDRF  = 0xf0
#define releaseCaps() DDRF  = 0x00
#define pullupsOff()  PORTF = 0x00

//#define DEBUG
//                 _      _    _
//   __ ____ _ _ _(_)__ _| |__| |___ ___
//   \ V / _` | '_| / _` | '_ \ / -_|_-<
//    \_/\__,_|_| |_\__,_|_.__/_\___/__/
//
uint16_t a1X, a1Y, a2X, a2Y;




//    ___      _
//   / __| ___| |_ _  _ _ __
//   \__ \/ -_)  _| || | '_ \
//   |___/\___|\__|\_,_| .__/
//                     |_|
void setup() {

  pullupsOff(); // turn off pullups
  holdCaps();   // start with caps discharged

  pinMode(joy1Button1 , INPUT_PULLUP);
  pinMode(joy1Button2 , INPUT_PULLUP);

  pinMode(joy2Button1 , INPUT_PULLUP);
  pinMode(joy2Button2 , INPUT_PULLUP);

  // Sends a clean report to the host. This is important on any Arduino type.
  Keyboard.begin();

#if defined (DEBUG)
  Serial.begin(9600);
#endif
}


//    __  __      _        _
//   |  \/  |__ _(_)_ _   | |   ___  ___ _ __
//   | |\/| / _` | | ' \  | |__/ _ \/ _ \ '_ \
//   |_|  |_\__,_|_|_||_| |____\___/\___/ .__/
//                                      |_|
void loop() {

  // Update Analog Axes
  sampleAnalogAxes();

  if (a1Y < 100) { // Brake presse
    Keyboard.press(KEY_V);
  } else if (a1Y > 300) { // Gas pressed
    Keyboard.press(KEY_R);
  } else {
    Keyboard.releaseAll();
  }


#if defined (DEBUG)
  Serial.print("a1Y:");Serial.print(a1Y);
  Serial.println();
#endif

  // Simple debounce
  delay(10);

}



//     __              _   _
//    / _|_  _ _ _  __| |_(_)___ _ _  ___
//   |  _| || | ' \/ _|  _| / _ \ ' \(_-<
//   |_|  \_,_|_||_\__|\__|_\___/_||_/__/
//
void sampleAnalogAxes() {
  static volatile uint16_t counter = 0;
  uint8_t  sample;

  a1X = 0;
  a1Y = 0;
  a2X = 0;
  a2Y = 0;

  releaseCaps();
  delayMicroseconds(5);

  // sample inputs
  counter = 1024;
  do  {
    sample = ~PINF;
    delayMicroseconds(3);
    sample >>= 4;
    a1X += sample & 1; // F4
    sample >>= 1;
    a1Y += sample & 1; // F5
    sample >>= 1;
    a2X += sample & 1; // F6
    sample >>= 1;
    a2Y += sample & 1; // F7

  } while (--counter);

  // reset caps
  holdCaps();
}
