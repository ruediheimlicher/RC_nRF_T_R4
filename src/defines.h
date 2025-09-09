//
//  text.h
//  DOG_LCD
//
//  Created by Ruedi Heimlicher on 22.11.2013.
//
//

#ifndef DEFINES_H
#define DEFINES_H

#define LOOPLED LED_BUILTIN
#define PRINTLED     4 // Board nRF_T_nano2
#define BLINKRATE 0x04FA

#define OSZI_A D7
#define OSZI_A_HI    digitalWrite(OSZI_A,HIGH)
#define OSZI_A_LO    digitalWrite(OSZI_A,LOW)
#define OSZI_A_TOGG    digitalWrite(OSZI_A,!digitalRead(OSZI_A))

#define TEST 0
#define CE_PIN 9
#define CSN_PIN 10

// EEPROM
#define EEPROMTASTE  5

#define EEPROM_WRITE 0
#define EEPROM_READ  1

#define EEPROMINDEX_U 0x10
#define EEPROMINDEX_O 0x20
#define EEPROMINDEX_M 0x30

#define EEPROMLEVELSETTINGS  0x40
#define EEPROMEXPOSETTINGS  0x48

// defines for PINS
// links
#define PITCH_PIN     A6
#define YAW_PIN       A3

// rechts
#define ROLL_PIN      A1
#define THROTTLE_PIN  A0  

#define BATT_PIN         A2
#define BUZZPIN 6

#define MINDIFF 4



// Tastatur
#define TASTATUR_PIN A7
#define TASTE_OFF  0
#define TASTE_ON  1

// Display
#define HOMESCREEN      0
#define MODELLSCREEN    1 // SYMPEL, TROTTLE ...
#define FUNKTIONSCREEN  2 // YAW,PITCH ...
#define AKTIONSCREEN    3 // LEVEL, EXPO ...


#define MODUSSCREEN     5

// 
#define  MODELL         0
#define  SIM            1
#define  CALIB          2

#define SAVE            1
#define CANCEL          0
#define CHANGED         2
#define CALIB_START     3
#define CALIB_END       4

#define arrow_width 16
#define arrow_height 16

// balken
#define VBX   64
#define VBY    12
#define HBX    6
#define HBY    54




/*
static unsigned char pfeil_l[] = {
   0b1,
   0b11,
   0b111,
   0b1111,
   0b11111,
   0b111111,
   0b1111111,
   0b11111111,
   0b1111111,
   0b111111,
   0b11111,
   0b1111,
   0b111,
   0b11,
   0b1


};
static unsigned char pfeil_r[] = 
{
   0b10000000,
   0b11000000,
   0b11100000,
   0b11110000,
   0b11111000,
   0b11111100,
   0b11111110,
   0b11111111,
   0b11111110,
   0b11111100,
   0b11111000,
   0b11110000,
   0b11100000,
   0b11000000,
   0b10000000
   
   };

   static unsigned char pfeil_d[] = {
  0x00, 0x00, // ................
  0x00, 0x00, // ................
  0x00, 0x00, // ................
  0x00, 0x00, // ................
  0x00, 0x00, // ................
  0x00, 0x00, // ................
  0x10, 0x00, // .......#........
  0x38, 0x00, // ......###.......
  0x7C, 0x00, // .....#####......
  0xFE, 0x00, // ....#######.....
  0xFF, 0x01, // ...#########....
  0xFF, 0x03, // ..###########...
  0xFF, 0x07, // .#############..
  0xFF, 0x0F, // ###############.
  0xFF, 0x1F, // ################
  0x00, 0x00  // ................
};
*/



   #define arrow_left_width 8
#define arrow_left_height 8

#endif


