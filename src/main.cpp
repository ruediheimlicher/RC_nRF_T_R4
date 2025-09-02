#include "arduino.h"
#include "main.h"
#include <SPI.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include "display.h"

#include "expo8.h"
#include <nRF24L01.h>
#include <RF24.h>
#include <Bounce2.h> // github.com/thomasfredericks/Bounce2

#include <elapsedMillis.h>
#include "defines.h"


const uint64_t pipeOut = 0xABCDABCD71LL;    


int led = LED_BUILTIN;

uint16_t loopcounter = 0;
uint16_t loopcounter1 = 0;
uint8_t blinkcounter = 0;
uint8_t impulscounter = 0;

// the setup routine runs once when you press reset:
void setup()

{
  Serial.begin(9600);
  delay(100);
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);


}

// the loop routine runs over and over again forever:
void loop() 
{
  loopcounter++;
  Serial.println(loopcounter);
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);               // wait for a second
}