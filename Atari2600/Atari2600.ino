/*
     ___         _           _   ___             _
    / __|___ _ _| |_ _ _ ___| | | __| _ ___ __ _| |__
   | (__/ _ \ ' \  _| '_/ _ \ | | _| '_/ -_) _` | / /
    \___\___/_||_\__|_| \___/_| |_||_| \___\__,_|_\_\

  https://hackaday.io/project/170908-control-freak

  Danjovic 2020

  Atari 2600 Controller - 25/04/2020
*/



//             _   _
//    ___ _ __| |_(_)___ _ _  ___
//   / _ \ '_ \  _| / _ \ ' \(_-<
//   \___/ .__/\__|_\___/_||_/__/
//       |_|
#define DEBUG 1


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

//                             AVR    Atari 2600
#define atariUp       A3 //    PF4      1 up
#define atariDown     A2 //    PF5      2 down
#define atariLeft     A1 //    PF6      3 left
#define atariRight    A0 //    PF7      4 right 
#define atariPaddle1  15 //    PB1      5 paddle 1

#define atariFire     14 //    PB3      6 fire
//                                      7 Vcc
//                                      8 GND
#define atariPaddle2  16 //    PB2      6 paddle 2


#define MAXCOUNTINGS 1023
#define TIMEOUT 1000     // in milliseconds


// control Capacitors charging
#define releaseCapacitors()   DDRB &= ~((1<<1)|(1<<2))
#define resetCapacitors()  do {DDRB|=((1<<1)|(1<<2)); PORTB&=~((1<<1)|(1<<2));} while (0)

enum controllerType {
  JOYSTICK = 0,
  STEERING,
  PADDLE,
  INVALID = 0xff
};


//                 _      _    _
//   __ ____ _ _ _(_)__ _| |__| |___ ___
//   \ V / _` | '_| / _` | '_ \ / -_|_-<
//    \_/\__,_|_| |_\__,_|_.__/_\___/__/
//
uint8_t combinedButtons = 0;
int16_t paddle1AnalogValue, paddle2AnalogValue;

int8_t incrementTable[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

//    ___      _
//   / __| ___| |_ _  _ _ __
//   \__ \/ -_)  _| || | '_ \
//   |___/\___|\__|\_,_| .__/
//                     |_|
void setup() {

  pinMode(atariUp    , INPUT_PULLUP);
  pinMode(atariDown  , INPUT_PULLUP);
  pinMode(atariLeft  , INPUT_PULLUP);
  pinMode(atariRight , INPUT_PULLUP);
  pinMode(atariFire  , INPUT_PULLUP);

  resetCapacitors();

  // Sends a clean report to the host. This is important on any Arduino type.
  Gamepad.begin();

#if defined (DEBUG)
  Serial.begin(9600);
  Serial.println("starting...");
#endif
}


//    __  __      _        _
//   |  \/  |__ _(_)_ _   | |   ___  ___ _ __
//   | |\/| / _` | | ' \  | |__/ _ \/ _ \ '_ \
//   |_|  |_\__,_|_|_||_| |____\___/\___/ .__/
//                                      |_|
void loop() {
  int16_t axis;
  uint8_t typeNow;

  unsigned long timeNow, deltaTime;
  static unsigned long lastTime = millis();

  static uint8_t lastType = JOYSTICK;
  static uint8_t lastButtons = 0;
  static int16_t drivingX = 0;
  static uint8_t wheelStep = 1;

  // Start with a clean slate
  Gamepad.releaseAll();

  // Read controller
  // Bit     7  6  5  4  3  2  1  0
  // Button  0  0  0  Fr RG LF DW UP

  typeNow = atari2600Scan();

  if (typeNow == lastType) {
    switch (typeNow) {
      
      case JOYSTICK:
        axis = 0;
        if (combinedButtons & (1 << 0)) axis -= (int16_t)127; // Up
        if (combinedButtons & (1 << 1)) axis += (int16_t)127; // Down
        Gamepad.yAxis(axis);

        axis = 0;
        if (combinedButtons & (1 << 2)) axis -= (int16_t)127; // Left
        if (combinedButtons & (1 << 3)) axis += (int16_t)127; // Right
        Gamepad.xAxis(axis);

        // Update Buttons
        if (combinedButtons & (1 << 4)) Gamepad.press(1);

#if defined (DEBUG)
        Serial.print("Stick: ");
        if (combinedButtons & (1 << 0)) Serial.print("U"); else Serial.print("-");
        if (combinedButtons & (1 << 1)) Serial.print("D"); else Serial.print("-");
        if (combinedButtons & (1 << 2)) Serial.print("L"); else Serial.print("-");
        if (combinedButtons & (1 << 3)) Serial.print("R"); else Serial.print("-");
        if (combinedButtons & (1 << 4)) Serial.print(" F"); else Serial.print(" -");
        Serial.println();
#endif
        break;


      case PADDLE:
        Gamepad.xAxis(paddle1AnalogValue);
        Gamepad.yAxis(paddle2AnalogValue);

        //(swapped directions/paddle buttons!)
        if (combinedButtons & (1 << 3)) Gamepad.press(1); // Left Paddle
        if (combinedButtons & (1 << 2)) Gamepad.press(2); // Right Paddle

#if defined (DEBUG)
        Serial.print("Pdl1: "); Serial.print(paddle1AnalogValue);
        if (combinedButtons & (1 << 3)) Serial.print(" F"); else Serial.print(" -");

        Serial.print(" / Pdl2: "); Serial.print(paddle1AnalogValue);
        if (combinedButtons & (1 << 2)) Serial.print(" F"); else Serial.print(" -");
        Serial.println();
#endif
        break;


      case STEERING:
        if (combinedButtons & (1 << 4)) Gamepad.press(1);

        if ( (combinedButtons & 0x03) != lastButtons) { // rotary encoder changed position?
          lastButtons <<= 2;
          lastButtons |= (combinedButtons & 0x03);
          lastButtons &= 0x0f;

          // modulate the value of wheelStep to reflect the velocity that the wheel is being spinned
          timeNow = millis();
          deltaTime = timeNow - lastTime;
          lastTime = timeNow;
          if (deltaTime < 2 )
            wheelStep = 8;
          else if (deltaTime < 6 )
            wheelStep = 4;
          //          else if (deltaTime < 8)
          //            wheelStep = 3;
          else wheelStep = 1;

          // positiove values are subtracted and negative values are added to compensate the inversion on
          // bit states during sample of the joystick port.
          if (incrementTable[lastButtons] > 0) { // Increment value
            drivingX -= wheelStep;
          } else if (incrementTable[lastButtons] < 0) { // decrement value
            drivingX += wheelStep;
          }

          if (drivingX > 127) drivingX = 127;
          if (drivingX < -127) drivingX = -127;
        }
        Gamepad.xAxis(drivingX);

#if defined (DEBUG)
        Serial.print("Wheel :"); Serial.print(drivingX);
        if (combinedButtons & (1 << 4)) Serial.print(" F"); else Serial.print(" -");
        Serial.println();        
#endif

        break;
    } // switch
  } else {    // type changed, do not update, wait for next change

    lastType = typeNow;

#if defined (DEBUG)
    Serial.print("changed :"); Serial.println(typeNow);
#endif
  }

  Gamepad.write();
  // Simple debounce. Should be turned off while sampling steering controller
  if (typeNow != STEERING) delay(10);
}



//     __              _   _
//    / _|_  _ _ _  __| |_(_)___ _ _  ___
//   |  _| || | ' \/ _|  _| / _ \ ' \(_-<
//   |_|  \_,_|_||_\__|\__|_\___/_||_/__/
//

//
//
//
uint8_t atari2600Scan(void) {
  uint8_t sample, controllerType;
  static uint8_t lastControllerType = JOYSTICK;
  static unsigned long lastTime;
  static bool timedOut;
  bool paddleEvidence, steeringEvidence;


  // Scan digital controls
  combinedButtons = 0;
  if (digitalRead(atariFire) == 0) combinedButtons |= (1 << 4); // Fire button

  sample = ~PINF;
  if (sample & (1 << 7)) combinedButtons |= (1 << 0); // up
  if (sample & (1 << 6)) combinedButtons |= (1 << 1); // down
  if (sample & (1 << 5)) combinedButtons |= (1 << 2); // left
  if (sample & (1 << 4)) combinedButtons |= (1 << 3); // right

  paddleEvidence = ( sampleAnalogValues()  ); // bith axes not timed out denotes a paddle controller
  steeringEvidence = ( (combinedButtons & (1 << 0)) && (combinedButtons & (1 << 1)) ); // UP+DOWN

  if (lastControllerType == STEERING) {// Steering controller is a game ender, only exits when a paddle is inserted or USB device reset
    if (paddleEvidence)
      controllerType = PADDLE;
  } else {
    if (steeringEvidence)
      controllerType = STEERING;
    else if (paddleEvidence)
      controllerType = PADDLE;
    else
      controllerType = JOYSTICK;
  }

  lastControllerType = controllerType;
  return controllerType;
}


//
// Read analog values by measuring time for capacitor to charge.
// return true if none of the channels timed out.
//
uint8_t sampleAnalogValues() {
  uint8_t sample;
  uint16_t countVar, aX, aY;

  uint8_t saveTCCR0B = TCCR0B;

  // stop timer 0 to prevent housekeeping during analog sampling
  TCCR0B &= ~( (1 << CS02) | (1 << CS01) | (1 << CS00) );

  // Release capacitors to charge
  releaseCapacitors();

  // now count time it takes for each input to flip HIGH
  aY = 0; aX = 0, countVar = MAXCOUNTINGS;
  do {
    sample = ~PINB;
    sample >>= 1;            // skip bit 0
    aX += sample & (1 << 0); // sample bit 1 Paddle 1, X
    sample >>= 1;
    aY += sample & (1 << 0); // sample bit 1 Paddle 2, Y
  } while (--countVar);

  resetCapacitors();  // reset external capacitors

  TCCR0B = saveTCCR0B; // resume Timer 0

  paddle1AnalogValue = (int16_t)aX;
  paddle2AnalogValue = (int16_t)aY;

  // Paddle is present when neither value reach maximum counting
  return ((aX != MAXCOUNTINGS) && (aY != MAXCOUNTINGS) ) ;
}
