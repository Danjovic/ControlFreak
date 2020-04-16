/*
   ___         _           _   ___             _
  / __|___ _ _| |_ _ _ ___| | | __| _ ___ __ _| |__
 | (__/ _ \ ' \  _| '_/ _ \ | | _| '_/ -_) _` | / /
  \___\___/_||_\__|_| \___/_| |_||_| \___\__,_|_\_\

  https://hackaday.io/project/170908-control-freak

  Danjovic 2020

 SEGA Controller - 16/04/2020
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

//                               AVR    Genesis
#define genesisRigth    A3 //    PF4      1
#define genesisLeft     A2 //    PF5      2
#define genesisDown     A1 //    PF6      3
#define genesisUp       A0 //    PF7      4 
//                                        5   Vcc
#define genesisB        16 //    PB3      6
#define genesisSelect   14 //    PB2      7
//                                        8   GND
#define genesisC        15 //    PB1      9




//#define DEBUG

enum controllerType {
  _6Button=0,
  _3Button,
  _unKnown =0xff
  };



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

  pinMode(genesisUp    ,INPUT_PULLUP);
  pinMode(genesisDown  ,INPUT_PULLUP);
  pinMode(genesisLeft  ,INPUT_PULLUP);
  pinMode(genesisRigth ,INPUT_PULLUP);
  pinMode(genesisB     ,INPUT_PULLUP);
  pinMode(genesisC     ,INPUT_PULLUP);
  pinMode(genesisSelect,OUTPUT);
  digitalWrite(genesisSelect,LOW);

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
  
  // Start with a clean slate
  Gamepad.releaseAll(); 


  // Read controller  
  // Bit     15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0   
  // Button  0  0  0  0  MD X  Y  Z  ST A  C  B  RG LF DW UP
  SEGAscan();
  axis=0;
  if (combinedButtons & (1<<0)) axis-=32767; // Up
  if (combinedButtons & (1<<1)) axis+=32767; // Down
  Gamepad.yAxis(axis);  

  axis=0;
  if (combinedButtons & (1<<2)) axis-=32767; // Left
  if (combinedButtons & (1<<3)) axis+=32767; // Right
  Gamepad.xAxis(axis);  

  // Update Buttons
  if (combinedButtons & (1<<6)) Gamepad.press(1);  // A
  if (combinedButtons & (1<<4)) Gamepad.press(2);  // B
  if (combinedButtons & (1<<5)) Gamepad.press(3);  // C

  if (combinedButtons & (1<<10))Gamepad.press(4);  // X
  if (combinedButtons & (1<<9)) Gamepad.press(5);  // Y
  if (combinedButtons & (1<<8)) Gamepad.press(6);  // Z
      
  if (combinedButtons & (1<<7)) Gamepad.press(7);  // Start
  if (combinedButtons & (1<<11))Gamepad.press(8);  // Mode

  // send the report to the host.
  Gamepad.write();

#if defined (DEBUG) 
  // Bit     15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0   
  // Button  0  0  0  0  MD X  Y  Z  ST A  C  B  RG LF DW UP

    Serial.print("Dpad: ");
    if (combinedButtons & (1<<0)) Serial.print ("U"); else Serial.print("-");
    if (combinedButtons & (1<<1)) Serial.print ("D"); else Serial.print("-");
    if (combinedButtons & (1<<2)) Serial.print ("L"); else Serial.print("-");
    if (combinedButtons & (1<<3)) Serial.print ("R"); else Serial.print("-");
        
    Serial.print(" Buttons: ");
    
    if (combinedButtons & (1<<6)) Serial.print ("A"); else Serial.print("-");
    if (combinedButtons & (1<<4)) Serial.print ("B"); else Serial.print("-");
    if (combinedButtons & (1<<5)) Serial.print ("C"); else Serial.print("-");
        
    if (combinedButtons & (1<<10)) Serial.print (" X"); else Serial.print(" -");
    if (combinedButtons & (1<<9)) Serial.print ("Y"); else Serial.print("-");
    if (combinedButtons & (1<<8)) Serial.print ("Z"); else Serial.print("-");
    
    if (combinedButtons & (1<<7)) Serial.print (" S"); else Serial.print(" -");
    if (combinedButtons & (1<<11)) Serial.print ("M"); else Serial.print("-");
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

//
// Read either SMS, 3button or 6 Button controller
// return 16 bit value  0  0  0  0  MD X  Y  Z  ST A  C  B  RG LF DW UP
uint8_t SEGAscan(void) {

  uint8_t sample[7]; 
  uint8_t type;  
  combinedButtons = 0;

  
//  delayMicroseconds(10);           //  5  4  3  2  1  0  -> BIT

  sample[0] = readController();           //  ST A  0  0  DW UP
  
  digitalWrite(genesisSelect,HIGH);  //  
  delayMicroseconds(10); 
  sample[1] = readController();           //  C  B  RG LF DW UP

  digitalWrite(genesisSelect,LOW);   //  
  delayMicroseconds(10);
  sample[2] = readController();           //  ST A  0  0  DW UP 
 
  digitalWrite(genesisSelect,HIGH);  //  
  delayMicroseconds(10);
  sample[3] = readController();           //  C  B  RG LF DW UP 

  digitalWrite(genesisSelect,LOW);   // 
  delayMicroseconds(10);
  sample[4] = readController();           //   ST A  0  0  0  0  -> 3 button: ST A  0  0  DW  UP 

  digitalWrite(genesisSelect,HIGH);  //  
  delayMicroseconds(10);
  sample[5] =  readController();           //  1  1  MD X  Y  Z  

  digitalWrite(genesisSelect,LOW);   //
  delayMicroseconds(10);
  sample[6] = readController();            //  ST A  1  1  1  1 -> 3 button:  ST A  0  0  DW  UP 
    

  // check for 3 or 6 buttons
  if ( ((sample[4] & 0x03) == 0) && ((sample[6] & 0x0f)==0x0f) ) {
  type = _6Button;  
  } else if  ( (sample[6] & 0x0c) == 0)  {
  type = _3Button;
  } else
    return _unKnown; // unknown

 
  // now populate combinedButtons variable accordingly       // 15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
  combinedButtons = (uint16_t)sample[1];                     // 0  0  0  0  0  0  0  0  0  0  C  B  RG LF DW UP
  combinedButtons |= ((uint16_t)(sample[0]<<2)) & 0xc0;      // 0  0  0  0  0  0  0  0  ST A  C  B  RG LF DW UP
  combinedButtons |= ((uint16_t)(sample[5]<<8)) & 0xf00;     // 0  0  0  0  MD X  Y  Z  ST A  C  B  RG LF DW UP

  // invert bits. Make '1' the active state 
  combinedButtons = ~combinedButtons;
  switch (type) {
    case _6Button:
      combinedButtons &= 0x0fff;
      break;

    case _3Button:
      combinedButtons &= 0x00ff;

    default:
      combinedButtons &= 0x003f;    
    } // case

  return type;
}


uint8_t readController() {

  uint8_t buttons=0;

  if( digitalRead(genesisUp)   ) buttons |= (1<<0);
  if( digitalRead(genesisDown) ) buttons |= (1<<1);
  if( digitalRead(genesisLeft) ) buttons |= (1<<2);
  if( digitalRead(genesisRigth)) buttons |= (1<<3);
  if( digitalRead(genesisB)    ) buttons |= (1<<4);
  if( digitalRead(genesisC)    ) buttons |= (1<<5);  
#if defined (DEBUG) 
   Serial.println(buttons | 128,BIN);
#endif  
  return buttons;
  }
