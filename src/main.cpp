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




uint16_t                   impulstimearray[NUM_SERVOS] = {};
const int                  adcpinarray[NUM_SERVOS] = {A3,A6,A1,A0};    // pins der Pots

uint8_t                    kanalsettingarray[ANZAHLMODELLE][NUM_SERVOS][KANALSETTINGBREITE] = {};
uint16_t                   servomittearray[NUM_SERVOS] = {}; // Werte fuer Mitte

uint16_t        potgrenzearray[NUM_SERVOS][2]; // obere und untere Grenze von adc
uint8_t         levelwert= 0;
uint8_t         expowert = 0;
uint16_t        potwert = 0;
uint8_t         expowertarray[NUM_SERVOS] = {}; // expowert pro Servo
uint16_t          potwertarray[NUM_SERVOS] = {}; // Werte fuer Mitte

uint16_t      blink_cursorpos=0xFFFF;

// statusvariablen
uint8_t blinkstatus = 0;
uint8_t curr_steuerstatus = 0;
uint8_t calibstatus = 0;
uint8_t savestatus = 0;

// timing
uint16_t stopsekunde=0;
uint16_t stopminute=0;
uint16_t motorsekunde=0;
uint16_t motorminute=0;
uint8_t motorstunde=0;

uint16_t sendesekunde=0;
uint16_t sendeminute=0;
uint8_t sendestunde=0;
uint8_t           sekundencounter = 0;
elapsedMillis   zeitintervall;
elapsedMillis   sinceLastBlink = 0;

// pot
uint16_t diffa = 0;
uint16_t diffb = 0;
uint16_t expoint = 0;
uint16_t levelint = 0;
uint16_t intdiff = 0;



// Batterie
uint16_t batteriespannung = 0;
uint16_t batteriearray[8] = {};
uint16_t batteriemittel = 0;
uint8_t batteriemittelwertcounter = 0;
uint16_t batterieanzeige = 0;
float UBatt = 0;

// EEPROM
uint8_t eepromstatus = 0;
uint16_t eepromprelltimer = 0;
Bounce2::Button eepromtaste = Bounce2::Button();
uint8_t taskarray[4] = {'Y', 'P', 'R', 'T'};

// Tastatur
uint16_t tastaturwert = 0;
uint8_t tastencounter = 0;
uint8_t tastaturstatus = 0;
uint8_t Taste = 0;
uint8_t taste5counter = 0; // Taste 5
uint8_t                 Tastenindex=0;
uint16_t                Tastenwert=0;
uint8_t                 adcswitch=0;
uint16_t                lastTastenwert=0;
int16_t                 Tastenwertdiff=0;
uint16_t                tastaturcounter=0;
uint16_t                tastaturdelaycounter=0;



// nRF
uint16_t errcounter = 0;
uint16_t radiocounter = 0;

// Display
uint16_t                cursorpos[8][8]={}; // Aktueller screen: werte fuer page und darauf liegende col fuer den cursor

unsigned        char char_x = 0;
unsigned          char char_y = 0;
// Menu
uint8_t                 curr_model=0; // aktuelles modell
uint8_t                 speichermodel=0;
uint8_t                 curr_funktion=0; // aktuelle funktion
uint8_t                 curr_aktion=0; // aktuelle aktion
uint8_t                  curr_wert = 0;
uint8_t                 curr_impuls=0; // aktueller impuls
uint8_t                 curr_modus=0; // Modell oder Sim oder Calib
uint8_t                 curr_setting=0; // aktuelles Setting fuer Modell
uint8_t                          speichersetting=0;
uint8_t                 curr_trimmkanal=0; // aktueller  Kanal fuerTrimmung
uint8_t                 curr_trimmung=0; // aktuelle  Trimmung fuer Trimmkanal
uint8_t                 curr_screen = 0; // aktueller screen
uint8_t                 last_screen=0; // letzter screen
uint8_t                 curr_page=7; // aktuelle page
uint8_t                 curr_col=0; // aktuelle colonne
uint8_t                 curr_cursorzeile=0; // aktuelle zeile des cursors
uint8_t                 curr_cursorspalte=0; // aktuelle colonne des cursors
uint8_t                 last_cursorzeile=0; // letzte zeile des cursors
uint8_t                 last_cursorspalte=0; // letzte colonne des cursors


Signal data;

// Functions
void ResetData() 
{
   data.throttle = 0;                  
   data.pitch = 127;
   data.roll = 127;
   data.yaw = 122;
   data.aux1 = 0;                       
   data.aux2 = 0;
   
}

// PPM decode
const byte PPM_PIN = 2; // PPM-Eingang an Pin 2
volatile unsigned long lastTime = 0;
volatile unsigned long pulseLength = 0;
volatile byte channel = 0;
const byte maxChannels = 8;
volatile unsigned int ppmValues[maxChannels];
void ppmISR() 
{
  unsigned long now = micros();
  pulseLength = now - lastTime;
  lastTime = now;

  if (pulseLength > 3000) {
    // Sync-Pause erkannt: neues Frame beginnt
    channel = 0;
  } else if (channel < maxChannels) {
    ppmValues[channel] = pulseLength;
    channel++;
  }
}

void printgrenzen()
{
   Serial.print("\nprintgrenzen\n");  
   for (uint8_t i = 0;i<NUM_SERVOS;i++)
   {
      Serial.print("grenzen i:\t");
      Serial.print(i);
      Serial.print("\t");
      Serial.write(taskarray[i]);
      Serial.print("\t");
      Serial.print("potgrenze HI:\t");
      Serial.print(potgrenzearray[i][0]);
      Serial.print("\t");
      Serial.print("potgrenze LO:\t");
      Serial.print(potgrenzearray[i][1]);
      Serial.print("\t");
      Serial.print("servomitte:\t");
      Serial.print(servomittearray[i]);
      Serial.print("\n");
   }
   Serial.print("end printgrenzen\n");  
}

void printeeprom(uint8_t zeilen)
{
   Serial.print("printeeprom\n");
   for (uint8_t i=0;i<zeilen;i++)
   {
      //Serial.write(taskarray[i]);
      //Serial.print("\t");
      uint8_t f = EEPROM.read(i);
      
      if ((i+1)%8==0 )
      {
         //Serial.print(i);
         //Serial.print(": ");
         
         Serial.print(f);
         Serial.print("\n");
      }
      else
      {
         //Serial.print(i);
         //Serial.print(": ");
         
         Serial.print(f);
         Serial.print("\t");
      }
      
   }
      Serial.print("\n");
   uint8_t eepromyawlo = EEPROM.read(2*(0 + EEPROMINDEX_U));
   uint8_t eepromyawhi = EEPROM.read(2*(0 + EEPROMINDEX_U)+1);
   uint16_t eepromyaw = (eepromyawhi << 8) | eepromyawlo;
   
   Serial.print("eeprompitch U: \t");
   Serial.print(eepromyawlo);
   Serial.print("\t");
   Serial.print(eepromyawhi);
   Serial.print("\t");
   Serial.print(eepromyaw);
   Serial.print("\n");
   
   eepromyawlo = EEPROM.read(2*(0 + EEPROMINDEX_O));
   eepromyawhi = EEPROM.read(2*(0 + EEPROMINDEX_O)+1);
   eepromyaw = (eepromyawhi << 8) | eepromyawlo;
   
   Serial.print("eepromyaw O: \t");
   Serial.print(eepromyawlo);
   Serial.print("\t");
   Serial.print(eepromyawhi);
   Serial.print("\t");
   Serial.print(eepromyaw);
   Serial.print("\n");
}

void eepromread()
{
   Serial.print("eepromread \t");
   //Serial.print("kontrolle Adresse A: ");
   //Serial.print(EEPROM.read(0));
   //Serial.print(" Adresse B: ");
   //Serial.println(EEPROM.read(0));
   for (uint8_t i = 0;i<NUM_SERVOS;i++)
   {
      Serial.write(taskarray[i]);
      Serial.print("\t");
      uint8_t l = (potgrenzearray[i][0] & 0x00FF); // lo byte
      uint8_t h = (potgrenzearray[i][0] & 0xFF00)>>8; // hi byte
      Serial.print("potgrenzearray 0\t");
      Serial.print(potgrenzearray[i][0]);
      Serial.print("\t");
      uint16_t grenzeU = (h << 8) | l;
      Serial.print("grenzeU\t");
      
      uint8_t el = 0;
      uint8_t  eh = 0;
      el = EEPROM.read(2*(i + EEPROMINDEX_U)); // lo byte
      eh = EEPROM.read(2*(i + EEPROMINDEX_U)+1); // hi byte
      Serial.print(el);
      Serial.print("\t");
      Serial.print(eh);
      Serial.print("\t");
      
      potgrenzearray[i][1] = (eh << 8) | el;
      
      el = EEPROM.read(2*(i + EEPROMINDEX_O)); // lo byte
      eh = EEPROM.read(2*(i + EEPROMINDEX_O)+1); // hi byte
      Serial.print(el);
      Serial.print("\t");
      Serial.print(eh);
      Serial.print("\t");
      
      potgrenzearray[i][0] = (eh << 8) | el;
      
      el = EEPROM.read(2*(i + EEPROMLEVELSETTINGS));
      kanalsettingarray[0][i][1] = el; // modell 0
      
      eh = EEPROM.read(2*(i + EEPROMEXPOSETTINGS));
      kanalsettingarray[0][i][2] = eh; // modell 0
      
      if(i==0)
      {
         Serial.print("\n");
         Serial.print("level\t");
         Serial.print(el);
         Serial.print("\t");
         Serial.print("expo\t");
         Serial.println(eh);

 
      }
      
      
      
      
   } // for i
   Serial.print("\n");  
}

void clearsettings(void)
{
   Serial.print("clearsettings\n");  
   for (uint8_t i = 0;i<NUM_SERVOS;i++)
   {
      kanalsettingarray[curr_model][i][1] = 0x00; // level
      kanalsettingarray[curr_model][i][2] = 0x00; // level
      
   } // for i
}

void cleargrenzen(void)
{
   Serial.print("cleargrenzen\n");  
   for (uint8_t i = 0;i<NUM_SERVOS;i++)
   {
      potgrenzearray[i][0] = 127; // 
      potgrenzearray[i][1] = 127; 
      
   } // for i
}

void eepromwrite(void)

{
   Serial.print("eepromwrite\n");  
   for (uint8_t i = 0;i<NUM_SERVOS;i++)
   {
      Serial.print("potgrenzearray raw i:\t");
      Serial.print(i);
      Serial.print("\t");
      Serial.write(taskarray[i]);
      Serial.print("\t");
      Serial.print("potgrenze HI:\t");
      Serial.print(potgrenzearray[i][0]);
      Serial.print("\t");
      Serial.print("potgrenze LO:\t");
      Serial.print(potgrenzearray[i][1]);
      Serial.print("\t");
      Serial.print("servomitte:\t");
      Serial.print(servomittearray[i]);
      Serial.print("\t");
      Serial.print("adresse U:\t");
      uint8_t addresseU_LO = 2*(i + EEPROMINDEX_U);
      Serial.print(addresseU_LO);
      Serial.print("\t");
      Serial.print("adresse H:\t");
      uint8_t addresseU_HI = 2*(i + EEPROMINDEX_U)+1;
      Serial.print(addresseU_HI);
      
      
      Serial.print("\n");
      
      
      
      EEPROM.update(2*(i + EEPROMINDEX_U),(potgrenzearray[i][1] & 0x00FF)); // lo byte
      EEPROM.update(2*(i + EEPROMINDEX_U)+1,((potgrenzearray[i][1] & 0xFF00) >> 8)); // hi byte
      
      EEPROM.update(2*(i + EEPROMINDEX_O),(potgrenzearray[i][0] & 0x00FF)); // lo byte
      EEPROM.update(2*(i + EEPROMINDEX_O)+1,((potgrenzearray[i][0] & 0xFF00) >> 8)); // hi byte
      
      
      EEPROM.update(2*(i + EEPROMINDEX_M),(servomittearray[i] & 0x00FF)); // lo byte
      EEPROM.update(2*(i + EEPROMINDEX_M)+1,((servomittearray[i] & 0xFF00) >> 8)); // hi byte
      
      
      for (uint8_t i=0;i<8;i++)
      {
         //     EEPROM.write(2*(i + EEPROMLEVELSETTINGS)+i,255); // lo byte
         //    EEPROM.write(2*(i + EEPROMEXPOSETTINGS)+i,255);
         
      }
      
      EEPROM.update(2*(i + EEPROMLEVELSETTINGS),(kanalsettingarray[curr_model][i][1] )); // level
      EEPROM.update(2*(i + EEPROMEXPOSETTINGS),(kanalsettingarray[curr_model][i][2] )); // expo
      
      //EEPROM.update(2*(i + EEPROMLEVELSETTINGS),(47+i)); // level
      //EEPROM.update(2*(i + EEPROMEXPOSETTINGS),(63+i )); // expo
      
      EEPROM.update(0,17);
      EEPROM.update(1,33);
      
      
      delay(20);
      /*
       Serial.print("kontrolle i: \t*");
       Serial.print(i);
       Serial.print("\t");
       uint8_t el = EEPROM.read(2*(i + EEPROMINDEX_U)); // lo byte
       uint8_t eh = EEPROM.read(2*(i + EEPROMINDEX_U)+1); // hi byte
       Serial.print(el);
       Serial.print("\t");
       Serial.print(eh);
       Serial.print("\t");
       uint16_t grenzeU = (eh << 8) | el;
       Serial.print("grenzeU eeprom:\t");
       Serial.print(grenzeU);
       Serial.print(" *\n");
       */
   }
   
   Serial.print("eepromwrite end\n");
}


// Tastatur

uint8_t Joystick_Tastenwahl(uint16_t Tastaturwert)
{
   //return 0;
   if (Tastaturwert < JOYSTICKTASTE1) 
      return 2;
   if (Tastaturwert < JOYSTICKTASTE2)
      return 1;
   if (Tastaturwert < JOYSTICKTASTE3)
      return 4;
   if (Tastaturwert < JOYSTICKTASTE4)
      return 7;
   if (Tastaturwert < JOYSTICKTASTE5)
      return 8;
   if (Tastaturwert < JOYSTICKTASTE6)
      return 3;
   if (Tastaturwert < JOYSTICKTASTE7)
      return 6;
   if (Tastaturwert < JOYSTICKTASTE8)
      return 9;
   if (Tastaturwert < JOYSTICKTASTE9)
      return 5;
   /*
    if (Tastaturwert < JOYSTICKTASTEL)
    return 10;
    if (Tastaturwert < JOYSTICKTASTE0)
    return 0;
    if (Tastaturwert < JOYSTICKTASTER)
    return 12;
    */
   return 0;
}
// tastenwahl
void tastenfunktion(uint16_t Tastenwert)
{  
   tastaturcounter++;   
   if (Tastenwert>10) // ca Minimalwert der Matrix
   {      
      //Serial.print(Tastenwert);
      //Serial.print("\t");
      //Serial.print(tastaturcounter);
      
      //Serial.print("\n");
      
      if (tastaturcounter>=400)   //   Prellen
      {        
         
         tastaturcounter=0x00;
         //Serial.println("Taste down");
         if (!(tastaturstatus & (1<<TASTE_OK))) // Taste noch nicht gedrueckt
         {
            
            //Serial.println(Tastenwert);
            //Taste = 0;
            
            //tastaturstatus |= (1<<TASTE_ON); // nur einmal   
            tastaturstatus |= (1<<TASTE_OK); // nur einmal   
            Taste= Joystick_Tastenwahl(Tastenwert);
            tastaturstatus |= (1<<AKTION_OK);
            if(OLED && Taste) // Taste und Tastenwert anzeigen
            {
               //oled_delete(0,62,20);
               //u8g2.setCursor(0,42);
               //u8g2.print(tastaturwert);
               //u8g2.print("T ");
               //u8g2.setCursor(0,62);
               //u8g2.print(Taste);
               
               //u8g2.sendBuffer(); 
               
            }
            
            
            //;
         }
         else // Taste neu gedrückt
         {
            /*
             Taste = 0;
             //tastaturstatus |= (1<<TASTE_ON); // nur einmal 
             
             Taste= Joystick_Tastenwahl(Tastenwert);
             tastaturstatus |= (1<<AKTION_OK);
             if(OLED && Taste) // Taste und Tastenwert anzeigen
             {
             oled_delete(0,62,40);
             u8g2.setCursor(0,62);
             //u8g2.print(tastaturwert);
             u8g2.print("T ");
             u8g2.print(Taste);
             
             u8g2.sendBuffer(); 
             
             }
             */
            
         }
      }
      
      
   }// if tastenwert
   else 
   {
      //if (tastaturstatus & (1<<TASTE_ON))
      {
         
         //tastaturstatus &= ~(1<<TASTE_OK);
      }
   }
      
}//tastenfunktion

void setModus(void)
{
   switch (curr_modus)
   {
      case MODELL:
      {
         eepromread();
      }break;
         
      case SIM:
      {
         // Joystick-Settings auf neutral stellen
         for (uint8_t i=0;i<NUM_SERVOS;i++)
         {
            //Serial.print(adcpinarray[i]);
            //Serial.print("\t");
            //Serial.print(servomittearray[i]);
            //Serial.print("\t");
            
            kanalsettingarray[0][i][1] = 0x00; // level
            kanalsettingarray[0][i][2] = 0x00; // expo
         }
      }break;

      case CALIB:
      {

      }break;
   }// switch curr_steuerstatus
}



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