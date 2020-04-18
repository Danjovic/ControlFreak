/*
   ___         _           _   ___             _
  / __|___ _ _| |_ _ _ ___| | | __| _ ___ __ _| |__
  | (__/ _ \ ' \  _| '_/ _ \ | | _| '_/ -_) _` | / /
  \___\___/_||_\__|_| \___/_| |_||_| \___\__,_|_\_\

  https://hackaday.io/project/170908-control-freak

  Danjovic 2020

  PC Joystick - 17/04/2020
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

// Pin Assignment
//   SNES                                       NES
//   /---\                        
//   | O | GND   (Brown)
//   | O |                                     +----\
//   | O |                      (Brown)   GND  | O   \
//   +---+                       (Red)   CLOCK | O  O |
//   | O | DATA  (Red)          (Orange)  DATA | O  O |
//   | O | LATCH (Yellow)       (Yellow) LATCH | O  O |
//   | O | CLOCK (Blue)                        +------+
//   | O | +5V   (White)
//   +---+

#define dataPin  A2
#define latchPin A1
#define clockPin A0

enum controllerType {
  SNES = 0,
  NES,
  NONE = 0xff
};

//#define DEBUG


//                 _      _    _
//   __ ____ _ _ _(_)__ _| |__| |___ ___
//   \ V / _` | '_| / _` | '_ \ / -_|_-<
//    \_/\__,_|_| |_\__,_|_.__/_\___/__/
//
uint16_t combinedButtons = 0;

//    ___      _
//   / __| ___| |_ _  _ _ __
//   \__ \/ -_)  _| || | '_ \
//   |___/\___|\__|\_,_| .__/
//                     |_|
void setup() {
  pinMode(dataPin , INPUT_PULLUP);
  pinMode(latchPin , OUTPUT);
  pinMode(clockPin , OUTPUT);
  digitalWrite (latchPin, LOW);
  digitalWrite (latchPin, LOW);

  // Sends a clean report to the host.
  Gamepad.begin();

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

  int16_t axis;

  // Start with a clean slate
  Gamepad.releaseAll();

  // Read controller
  // Bit     15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
  // Button  0  0  0  0  Sl St R  L  Y  X  B  A  RG LF DW UP
  sNEScan();
  axis = 0;
  if (combinedButtons & (1 << 0)) axis -= 32767; // Up
  if (combinedButtons & (1 << 1)) axis += 32767; // Down
  Gamepad.yAxis(axis);

  axis = 0;
  if (combinedButtons & (1 << 2)) axis -= 32767; // Left
  if (combinedButtons & (1 << 3)) axis += 32767; // Right
  Gamepad.xAxis(axis);

  // Update Buttons
  if (combinedButtons & (1 << 4 )) Gamepad.press(1); // A
  if (combinedButtons & (1 << 5 )) Gamepad.press(2); // B

  if (combinedButtons & (1 << 6 )) Gamepad.press(3); // X
  if (combinedButtons & (1 << 7 )) Gamepad.press(4); // Y

  if (combinedButtons & (1 << 8 )) Gamepad.press(5); // L
  if (combinedButtons & (1 << 9 )) Gamepad.press(6); // R

  if (combinedButtons & (1 << 10)) Gamepad.press(7); // Start
  if (combinedButtons & (1 << 11)) Gamepad.press(8); // Select

  // send the report to the host.
  Gamepad.write();

#if defined (DEBUG)
  // Bit     15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
  // Button  0  0  0  0  Sl St R  L  Y  X  B  A  RG LF DW UP

  Serial.print("Dpad: ");
  if (combinedButtons & (1 << 0)) Serial.print ("U"); else Serial.print("-");
  if (combinedButtons & (1 << 1)) Serial.print ("D"); else Serial.print("-");
  if (combinedButtons & (1 << 2)) Serial.print ("L"); else Serial.print("-");
  if (combinedButtons & (1 << 3)) Serial.print ("R"); else Serial.print("-");

  Serial.print(" Buttons: ");

  if (combinedButtons & (1 << 4)) Serial.print ("A"); else Serial.print("-");
  if (combinedButtons & (1 << 5)) Serial.print ("B"); else Serial.print("-");
  if (combinedButtons & (1 << 6)) Serial.print (" X"); else Serial.print(" -");
  if (combinedButtons & (1 << 7)) Serial.print ("Y"); else Serial.print("-");

  if (combinedButtons & (1 << 8)) Serial.print (" L"); else Serial.print(" -");
  if (combinedButtons & (1 << 9)) Serial.print ("R"); else Serial.print("-");

  if (combinedButtons & (1 << 10)) Serial.print (" ST"); else Serial.print(" --");
  if (combinedButtons & (1 << 11)) Serial.print (" SL"); else Serial.print(" --");
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

// Pulse clock line.
inline void Pulse_clock() {
  delayMicroseconds(6);
  digitalWrite(clockPin, HIGH);
  delayMicroseconds(6);
  digitalWrite(clockPin, LOW);
}

// Get buttons state.
uint8_t sNEScan(void) {
  uint8_t i, controllerType;
  uint16_t dataIn;

  //latch buttons state. After latching, first button is ready at data output
  digitalWrite(latchPin, HIGH);
  delayMicroseconds(12);
  digitalWrite(latchPin, LOW);
  digitalWrite(clockPin, LOW);

  // read 16 bits from the controller
  dataIn = 0;
  for (i = 0; i < 16; i++) {
    dataIn >>= 1;
    if (digitalRead(dataPin) == 0) dataIn |= (1 << 15) ; // shift in one bit
    Pulse_clock();
  }

  if (digitalRead(dataPin)) { // 17th bit received should be a zero if an original controller is attached
    controllerType = NONE;
  } else {
    if ( (dataIn & 0xf000) == 0x0000)
      controllerType = SNES;
    else  if ( (dataIn & 0xff00) == 0xff00)
      controllerType = NES;
    else
      controllerType = NONE;
  }

  // Return Clock signal to idle level.
  digitalWrite(clockPin, HIGH);


#if defined (DEBUG)
  Serial.println(dataIn, BIN);
  Serial.print("Type: ");
  switch (controllerType) {
    case SNES:
      Serial.println("SNES");
      break;
    case NES:
      Serial.println("NES");
      break;
    default:
      Serial.println("Unknown/No controller");
      break;
  }
#endif



  // Bit     15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
  // Button  0  0  0  0  Sl St R  L  Y  X  B  A  RG LF DW UP

  combinedButtons = 0;

  // Common buttons
  if (dataIn & (1 << 2 )) combinedButtons |= (1 << 11); // Select
  if (dataIn & (1 << 3 )) combinedButtons |= (1 << 10); // Start
  if (dataIn & (1 << 4 )) combinedButtons |= (1 << 0 ); // Up
  if (dataIn & (1 << 5 )) combinedButtons |= (1 << 1 ); // Down
  if (dataIn & (1 << 6 )) combinedButtons |= (1 << 2 ); // Left
  if (dataIn & (1 << 7 )) combinedButtons |= (1 << 3 ); // Right


  if (controllerType == SNES)  {
    if (dataIn & (1 << 8 )) combinedButtons |= (1 << 4 ); // A
    if (dataIn & (1 << 0 )) combinedButtons |= (1 << 5 ); // B
    if (dataIn & (1 << 9 )) combinedButtons |= (1 << 6 ); // X
    if (dataIn & (1 << 1 )) combinedButtons |= (1 << 7 ); // Y
    if (dataIn & (1 << 10)) combinedButtons |= (1 << 8 ); // L
    if (dataIn & (1 << 11)) combinedButtons |= (1 << 9 ); // R

  } else { //  NES / Knockoff
    if (dataIn & (1 << 0 )) combinedButtons |= (1 << 4 ); // A
    if (dataIn & (1 << 1 )) combinedButtons |= (1 << 5 ); // B
  }

  return controllerType;
}

//    ___         _               _   ___      _        _ _
//   | _ \_ _ ___| |_ ___  __ ___| | |   \ ___| |_ __ _(_) |___
//   |  _/ '_/ _ \  _/ _ \/ _/ _ \ | | |) / -_)  _/ _` | | (_-<
//   |_| |_| \___/\__\___/\__\___/_| |___/\___|\__\__,_|_|_/__/
//
/*  NES/SNES Controller Protocol

                 ____
     Latch  ____/    \________________________________________________________________________________________________________
            ____________    __    __    __    __    __    __    __    __    __    __    __    __    __    __    __    _______
     Clock              \__/  \__/  \__/  \__/  \__/  \__/  \__/  \__/  \__/  \__/  \__/  \__/  \__/  \__/  \__/  \__/
            _________                                                                                                     _____
  SNES Data            [  B ][  Y ][ Sl ][ St ][ UP ][DOWN][LEFT][RIGH][  A ][  X ][TopL][TopR][  1 ][  1 ][  1 ][ 1 ][  0 ]
  NES  Data            [  A ][  B ][ Sl ][ St ][ UP ][DOWN][LEFT][RIGH][  0 ][  0 ][  0 ][  0 ][  0 ][  0 ][  0 ][ 0 ][  0 ]
  bit                     0     1    2     3     4     5     6     7     8     9     10    11    12    13    14    15   16(extra)
*/
