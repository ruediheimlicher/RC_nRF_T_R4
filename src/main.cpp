//#include "arduino.h"
#include "main.h"
#include <SPI.h>
#include <EEPROM.h>
//#include <U8g2lib.h>

#include "display.h"

#include "expo8.h"
#include <nRF24L01.h>
#include <RF24.h>
#include <Bounce2.h> // github.com/thomasfredericks/Bounce2

#include <elapsedMillis.h>
#include "defines.h"



const uint64_t pipeOut = 0xABCDABCD71LL;    


int led = LED_BUILTIN;

uint16_t loopcounter0 = 0;
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
uint8_t       levelwertarray[NUM_SERVOS] = {}; // leelwert pro servo
uint16_t      blink_cursorpos=0xFFFF;


RF24 radio(CE_PIN, CSN_PIN);


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

uint8_t levelintcheck = 0;


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
uint8_t tastaturstatus = 0;
uint16_t tastendelaycounter = 0;
uint8_t Taste = 0;
uint8_t oldTaste = 0;

uint8_t                    taste5counter = 0; // Taste 5
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
   if (Tastenwert>20) // ca Minimalwert der Matrix
   {      
      //Serial.print("\ntastenfunktion Tastenwert: ");
      //Serial.print(Tastenwert);
      //Serial.print("\t");
      //Serial.print(tastaturcounter);
      
      //Serial.print("\n");
      
      if (tastaturcounter>=400)   //   Prellen
      {        
         
         tastaturcounter=0x00;
         //Serial.println("Taste down");
         //tastaturstatus |= (1<<TASTE_OK);
         if (!(tastaturstatus & (1<<TASTE_OK))) // Taste noch nicht gedrueckt
         {
            Serial.print("Taste down ");
            //Serial.println(Tastenwert);
            //Taste = 0;
            
            //tastaturstatus |= (1<<TASTE_ON); // nur einmal   
            tastaturstatus |= (1<<TASTE_OK); // nur einmal   
            tastaturstatus |= (1<<TASTATUR_WAIT); // Warten
            tastendelaycounter = TASTENDELAY;

            Taste= Joystick_Tastenwahl(Tastenwert);
            Serial.print("\ntastenfunktion Tastenwert: ");
            Serial.print(Tastenwert);
            Serial.print("\t Taste: ");
            Serial.println(Taste);

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
         //else // Taste neu gedrückt
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

   //else 
   {
      //tastaturwert = 0;
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

int Throttle_Map255(int val, int fromlow, int fromhigh,int tolow, int tohigh, bool reverse)
{
   val = constrain(val, fromlow, fromhigh);
   val = map(val, fromlow,fromhigh, tolow, tohigh);

   uint8_t levelwerta = levelwertarray[THROTTLE] & 0x07;
   uint8_t levelwertb = (levelwertarray[THROTTLE] & 0x70)>>4;

   uint8_t expowerta = expowertarray[THROTTLE] & 0x07;

   uint16_t expoint = 3;
   uint16_t levelint = 0;

   expoint = expoarray8[expowerta][val];
   levelint = expoint * (8-levelwerta);
   levelint /= 4;



   return ( reverse ? 255 - levelint : levelint );
}


int Border_Mapvar255(uint8_t servo, int val, int lower, int middle, int upper, bool reverse)
{
   val = constrain(val, lower, upper); // Grenzen einhalten
   uint8_t levelwerta = levelwertarray[servo] & 0x07;
   uint8_t levelwertb = (levelwertarray[servo] & 0x70)>>4;

   uint8_t expowerta = expowertarray[servo] & 0x07;
   uint8_t expowertb = (expowertarray[servo] & 0x70)>>4;
  
  //levelwerta = 0;
  //levelwertb = 0;
  //expowerta = 0;
  //expowertb = 0;
  
   if ( val < middle )
   {
      
      val = map(val, lower, middle, 0, 127); // normieren auf 0-127
      //intdiff = val;
      intdiff =  (127 - val);// Abweichung von mitte, 
      //levelintraw = intdiff;
      //diffa = map(intdiff,0,(middle - lower), 0,512);
      diffa = intdiff;
      
      expoint = expoarray8[expowerta][diffa];
      levelint = expoint * (8-levelwerta);
      levelint /= 8;
      levelintcheck = 127 + levelint;
      levelint = 127 + levelint;
   }  
   else
   {
      val = map(val, middle, upper, 128, 255); // normieren auf 128 - 255
      //intdiff = val;
      
      intdiff =  (val - 127);// Abweichung von mitte, 
      //diffb = map(intdiff,0,(upper - middle),0,512);
      diffb = intdiff;
      if(diffb >= 127 )
      {
         diffb = 127;
      }
      expoint = expoarray8[expowertb][diffb];
      levelint = expoint * (8-levelwertb) ;     
      levelint /= 8;
      levelintcheck= 127 - levelint;
      levelint= 127 - levelint;
   }
   
   return ( reverse ? 255 - levelint : levelint );
}






// the setup routine runs once when you press reset:
void setup()

{
  Serial.begin(9600);
  delay(100);
  // initialize the digital pin as an output.
  pinMode(LOOPLED, OUTPUT);
  pinMode(PRINTLED, OUTPUT);

  // PPM decode
   pinMode(PPM_PIN, INPUT);
   attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);
   
   curr_steuerstatus = MODELL;

  Serial.println(__DATE__);
  Serial.println(__TIME__);
  printeeprom(160);
   
  eepromread();
  pinMode(BUZZPIN,OUTPUT);
  pinMode(BATT_PIN,INPUT);
   
  pinMode(TASTATUR_PIN,INPUT);
   
   
   //pinMode(EEPROMTASTE,INPUT_PULLUP);
   eepromtaste.attach( EEPROMTASTE ,  INPUT_PULLUP ); 
   eepromtaste.interval(5);
   eepromtaste.setPressedState(LOW);
   
  // Display
  initDisplay();
  delay(100);

  oled_vertikalbalken(BATTX,BATTY,BATTB,BATTH);
  delay(100);
  setHomeScreen();
  u8g2.sendBuffer(); 
  Serial.println("nach setup");

// Radio
   radio.begin();
   radio.openWritingPipe(pipeOut);
   radio.setChannel(124);
   radio.setAutoAck(false);
   //radio.setDataRate(RF24_250KBPS);    // The lowest data rate value for more stable communication  | Daha kararlı iletişim için en düşük veri hızı.
   radio.setDataRate(RF24_2MBPS); // Set the speed of the transmission to the quickest available
   
   
   radio.setPALevel(RF24_PA_MAX);      // Output power is set for maximum range  |  Çıkış gücü maksimum menzil için ayarlanıyor.
   
   radio.setPALevel(RF24_PA_MIN); 
   radio.setPALevel(RF24_PA_MAX); 
   
   radio.stopListening();              // Start the radio comunication for Transmitter | Verici için sinyal iletişimini başlatır.
   if (radio.failureDetected) 
   {
      radio.failureDetected = false;
      delay(250);
      Serial.println("Radio failure detected, restarting radio");
   }
   else
   {
      Serial.println("Radio OK");
   }
   ResetData();

   Serial.print("servomitte\n");
   for (uint8_t i=0;i<NUM_SERVOS;i++)
   {
      uint16_t wert = 500 + i * 50;
      wert = 750;
      impulstimearray[i] = wert; // mittelwert
      
      //potgrenzearray[i][0] = potlo;
      //potgrenzearray[i][1] = pothi;
      
      servomittearray[i] = analogRead(adcpinarray[i]);
      Serial.print("i:\t");
      Serial.print(i);
      Serial.print("\t");
      Serial.print(servomittearray[i]);
      Serial.print("\t lo: ");
      Serial.print(servomittearray[i] & 0x00FF);
      Serial.print("\t hi: ");
      Serial.print((servomittearray[i]& 0xFF00) >> 8);
      Serial.print("\n");
      
      
      //uint8_t n = i*i+1;
      //EEPROM.write(i,0 );
      
   }




  
   
   

} // end setup

// the loop routine runs over and over again forever:
void loop() 
{
  if((loopcounter0 % 128) == 0)
  {
   if(tastaturstatus & (1<<TASTATUR_WAIT)) // noch warten
   {
      if(tastendelaycounter)
      {
         tastendelaycounter--;

      }
      else // Tastatur lesen
      {
         tastaturstatus &= ~(1<<TASTATUR_WAIT);
         
      }
   }
   else
   {
      tastaturwert = analogRead(TASTATUR_PIN);
      tastenfunktion(tastaturwert);
   }


    


  }// if %64

  if (tastaturstatus & (1<<TASTE_OK) && Taste) // Menu ansteuern
   {
      //Taste = 0;
       tastaturcounter = 0;
      switch (Taste)
      {
         case 0: // null-pos, nichts tun
         {
            
         }break;
         case 1:
         {
            Serial.print("T 1");   
            switch (curr_screen)
            {
               case 1: //MENUSCREEN
               {
                  curr_screen = 5;
                  curr_cursorspalte = 0;
                  setModusScreen();
                  u8g2.sendBuffer();
                  
               }break;
            }     // switch curr_screen  
         }break;
            
         case 2: // UP
         {
            //Serial.print("T 2");
            if (tastaturstatus & (1<<AKTION_OK))
            {
               Serial.print("T 2 up*");
               tastaturstatus &=  ~(1<<AKTION_OK);
               tastaturstatus |= (1<<UPDATE_OK);
               switch (curr_screen)
               {
                  case 0: // HOMESCREEN
                  {
                     
                  }break;
                  case 1: // T2 MENUSCREEN
                  {
                     if(curr_model)
                     {
                        curr_model--;
                        updateMenuScreen();
                        u8g2.sendBuffer();
                     }
                  }
                  case 2: //T2 MODELLSCREEN
                  {
                     if(curr_funktion)
                     {
                        curr_funktion--;
                        updateModellScreen();
                        u8g2.sendBuffer();
                     }
                  }break;
                  case 3: //FUNKTIONSCREEN
                  {
                     switch (curr_cursorspalte)
                     {
                        case 0:
                        {
                           if(curr_aktion)
                           {
                              curr_aktion--;
                              updateFunktionScreen();
                              u8g2.sendBuffer();
                           }
                        }break;
                        case 1: // Level, expo up, down
                        {
                           Serial.print("curr_aktion: ");
                           Serial.print(curr_aktion) ;
                        }break;
                     }// switch curr_cursorspalte                     
                  }break;
                     
                  case 4: // T2 AKTIONSCREEN
                  {
                     switch (curr_cursorspalte)
                     {
                        case 0:
                        {
                           if(curr_wert )
                           {
                              curr_wert--;                              
                           }
                           
                        }break;
                        case 1: // T2 UP DOWN
                        {
                           uint8_t level = kanalsettingarray[curr_model][curr_funktion][1];
                           uint8_t levelO = (level & 0xF0) >> 4;
                           uint8_t levelU = (level & 0x0F);
                           
                           uint8_t expo = kanalsettingarray[curr_model][curr_funktion][2];
                           uint8_t expoO = (expo & 0xF0) >> 4;
                           uint8_t expoU = (expo & 0x0F);
                           
                           switch (curr_aktion)
                           {
                              case 0: //LEVEL
                              {
                                 switch (curr_wert)
                                 {
                                    case 0: // UP
                                    {
                                       if(levelO < 4)
                                       {
                                          levelO++;
                                          savestatus = CHANGED;;
                                       }
                                    }break;
                                    case 1: // DOWN
                                    {
                                       if (levelU < 4)
                                       {
                                          levelU++;
                                          savestatus = CHANGED;;
                                       }
                                    }break;
                                 } // switch curr_wert
                                 level = (levelO << 4) | levelU;
                                 kanalsettingarray[curr_model][curr_funktion][1] = level; 
                              }break;
                                 
                              case 1: // EXPO
                              {
                                 switch (curr_wert)
                                 {
                                    case 0: // UP
                                    {
                                       if(expoO < 4)
                                       {
                                          expoO++;
                                          savestatus = CHANGED;;
                                       }
                                    }break;
                                    case 1: // DOWN
                                    {
                                       if (expoU < 4)
                                       {
                                          expoU++;
                                          savestatus = CHANGED;;
                                       }
                                    }break;
                                 }
                                 expo = (expoO << 4) | expoU;
                                 kanalsettingarray[curr_model][curr_funktion][2] = expo; 
                                 
                              }break;
                                 
                           }// switch curr_aktion                        
                        }break;
                           
                     }// switch curr_cursorspalte
                     updateAktionScreen();
                     u8g2.sendBuffer();
                  }break;
                     
                  case 5: // T2 UP  MODUSSCREEN
                  {
                     if(curr_modus )
                     {
                        
                        switch (curr_modus)
                        {
                           case MODELL:
                           {
                              
                           }break;

                           case SIM:
                           {

                           }break;

                           case CALIB:
                           {
                              calibstatus &= ~(1<<CALIB_START);
                           }break;
                        }// switch curr_modus
                        curr_modus--;
                     
                        
                     }
                  }break;
               }// switch (curr_screen)
            }           
         }break;
            
         case 3:
         {
            Serial.print("T 3");   
            switch (curr_screen)
            {
               case 0: // HOMESCREEN
               {
                  // EEPROM lesen
                  eepromread();
                  printeeprom(160);
               }break;
                  
            } // switch curr_screen
            
         }break;
            
         case 4: // LEFT
         {
            //Serial.print("T 4");            
            if (tastaturstatus & (1<<AKTION_OK))
            {
               Serial.print("T 4 left");
               tastaturstatus &=  ~(1<<AKTION_OK);
               tastaturstatus |= (1<<UPDATE_OK);
               switch (curr_screen)
               {
                  case 0: // HOMESCREEN // Umschalten Simulator/Modell, TO DO
                  {
                     //if (savestatus == CHANGED)
                     {
                        if(curr_cursorspalte == 1)
                        {
                           curr_cursorspalte = 0; // Rahmen auf YES
                           
                           updateHomeScreen();
                           
                        }
                     }
                     
                  }break;
                  case 1: // MENUSCREEN
                  {
                     
                  }break;
                  case 2: // MODELLSCREEN
                  {
                     
                  }break;
                  case 3: // FUNKTIONSCREEN
                  {
                     switch (curr_cursorspalte)
                     {
                        case 0: 
                        {
                           updateFunktionScreen();
                           u8g2.sendBuffer();
                        }break;
                        case 1: // up, down enabled
                        {
                           curr_cursorspalte--;
                        }break;
                     }// switch curr_cursorspalte
                  }break;
                     
                  case 4: // AKTIONSCREEN
                  {
                     Serial.print("T4 case 4: curr_screen: ");
                     Serial.println(curr_screen);
                     curr_cursorspalte = 0;
                     curr_wert = 0;
                     updateAktionScreen();
                     u8g2.sendBuffer();
                  }break;
                     
                  case 5: // T4 LEFT ModusScreen
                  {
                     curr_screen = 1; // MENUSCREEN
                     setModus();
                     setMenuScreen();
                     updateMenuScreen();
                     u8g2.sendBuffer();
                     
                     
                  }break;
               }// swich curr_screen
               
            }
         }break;
            
         case 5: // Ebene tiefer
         {
            if (tastaturstatus & (1<<AKTION_OK))
            {
               Serial.print("T 5 in ");
               
               if ((curr_screen == 0) ) //&& (taste5counter < 3))
               {
                  tastaturstatus |= (1<<T5_WAIT); // Warten auf 3 Impulse
                  {               
                     taste5counter++;
                     Serial.print("taste5counter: ");
                     Serial.println(taste5counter);
                     tastaturcounter = 300; // Mehrfachklick ermoeglichen
                     if (taste5counter == 3)
                     {
                        curr_screen = 1;
                        taste5counter = 0;
                        tastaturstatus |= ~(1<<T5_WAIT); // Warten beendet
                        Serial.print("T5 setMenuScreen ");
                        //u8g2.clear();
                        
                        setMenuScreen();
                        u8g2.sendBuffer();
                     }
                  }
               }
               else //if (!(tastaturstatus & (1<<T5_WAIT))) // kein Warten
               {
                  taste5counter = 0;
                  if(curr_screen < 6)
                  {
                     Serial.print("T 5 klick ");
                     Serial.println(curr_screen);
                     switch (curr_screen)
                     {
                        case 1: // MODELLSCREEN
                        {
                           Serial.print("T 5 > Modellscreen curr_model: ");
                           Serial.println(curr_model);
                           setModellScreen();
                           curr_screen = 2;
                           u8g2.sendBuffer();
                        }break;
                        case 2: // FUNKTIONSCREEN
                        {
                           Serial.print("T 5 > FunktionScreen curr_funktion: " );
                           Serial.println(curr_funktion);
                           setFunktionScreen();
                           curr_screen = 3;
                           u8g2.sendBuffer();
                        }break;
                        case 3: // AKTIONSCREEN
                        {
                           Serial.print("T 5 > AktionScreen curr_aktion: " );
                           Serial.println(curr_aktion);
                           setAktionScreen();
                           curr_screen = 4;
                           u8g2.sendBuffer();
                        }break;
                        case 4: 
                        {
                           Serial.print("T 5 screen 4 curr_wert: ");
                           Serial.println(curr_wert);
                        }break;

                        case 5: // 
                        {
                           Serial.print("T 5 screen 5 curr_modus: ");
                           Serial.println(curr_modus);
                           
                           if(!(calibstatus & (1<<CALIB_START))) // calib noch nicht gesetzt
                           {
                              cleargrenzen();
                              calibstatus |= (1<<CALIB_START);
                           }
                           else
                           {
                              calibstatus &= ~(1<<CALIB_START);// calib beenden
                              eepromwrite();       // settings in eeprom
                           }
                           
                           
                           updateModusScreen();
                           u8g2.sendBuffer();

                           //setCalib();
                        }break;


                     }// switch (curr_screen)
                  }
               }
               Serial.print("T5 end: curr_screen: ");
               Serial.println(curr_screen);              
               tastaturstatus &=  ~(1<<AKTION_OK);
               tastaturstatus |= (1<<UPDATE_OK);  
            }
         }break;
            
         case 6: // RIGHT
         {
            Serial.print("T 6");
            if (tastaturstatus & (1<<AKTION_OK))
            {
               Serial.println("T 6 right");
               tastaturstatus &=  ~(1<<AKTION_OK);
               tastaturstatus |= (1<<UPDATE_OK);
               switch (curr_screen)
               {
                  case 0: // HOMESCREEN // Umschalten Simulator/Modell, TO DO
                  {
                     if (savestatus == CHANGED)
                     {
                        if(curr_cursorspalte == 0)
                        {
                           curr_cursorspalte = 1; // Rahmen auf NO
                           updateHomeScreen();
                           
                        }
                     }
                     
                  }break;
                  case 1: // MENUSCREEN , nach Modussrreen
                  {
                     {
                        setModusScreen();
                        curr_cursorzeile = 0;
                        curr_cursorspalte = 0;
                        curr_screen = 5; // MODUSSCREEN
                        u8g2.sendBuffer();
                     }
                  }break;
                  case 2: // MODELLSCREEN
                  {
                     
                  }break;
                  case 3: // FUNKTIONSCREEN
                  {
                     Serial.print("T6 case 3: curr_screen: ");
                     Serial.println(curr_screen);
                     switch (curr_cursorspalte)
                     {
                        case 0: 
                        {
                           curr_cursorspalte++; // max 1                        
                           updateFunktionScreen();
                           u8g2.sendBuffer();
                        }break;
                        case 1: // up, down enabled
                        {
                           switch (curr_aktion)
                           {
                              case 0: // Level
                              {
                                 uint8_t level = kanalsettingarray[curr_model][curr_funktion][1];
                                 uint8_t levelO = (level & 0xF0) >> 4;
                                 uint8_t levelU = (level & 0x0F);
                                 //if(curr_pfeil == PFEIL_UP)
                                 {
                                    
                                    //blink_cursorpos = 86<<8 | 22;
                                    //u8g2.setDrawColor(1);
                                    //u8g2.drawFrame(88,char_y,48,16);
                                 }                             
                              }break;
                              case 1: // expo
                              {
                                 uint8_t expo = kanalsettingarray[curr_model][curr_funktion][2];
                                 uint8_t expoO = (expo & 0xF0) >> 4;
                                 uint8_t expoU = expo & 0x0F;
                                 
                                 
                              }break;
                           }// switch curr_aktion
                        }break;
                     }// switch curr_cursorspalte
                  }break;
                  case 4: // AKTIONSCREEN
                  {
                     Serial.print("T6 case 4: curr_screen: ");
                     Serial.println(curr_screen);
                     switch (curr_cursorspalte)
                     {
                        case 0:
                        {
                           curr_cursorspalte = 1;
                        }break;
                     }// switch
                     updateAktionScreen();
                     u8g2.sendBuffer();
                  }break;
                     
                  case 5: // T6 ModusScreen
                  {
                     Serial.print("T6 case 5 ModusScreen: curr_screen: ");
                     Serial.println(curr_screen);
                     switch (curr_cursorzeile)
                     {
                        case 0: // Navigation
                        {
                           if(curr_cursorspalte)
                           {
                              curr_cursorspalte++;
                              
                              updateMenuScreen();
                              u8g2.sendBuffer();
                           }
                        }break;
                        case 1: // Auswahl
                        {
                           curr_steuerstatus = SIM;
                           setModus();
                        }break;
                     }// switch curr_cursorzeile
                  }break;
                     
               }// swich curr_screen  
            }
         }break;
            
         case 7:
         {
            if (tastaturstatus & (1<<AKTION_OK))
            {
               Serial.print("T 7 back ");
               if(curr_screen )
               {
                  curr_screen--;
                  u8g2.clear();
                  switch (curr_screen)
                  {
                     case 0: // HOMESCREEN
                     {
                        //savestatus = CHANGED;
                        setHomeScreen();
                     }break;
                     case 1: // MENUSCREEN
                     {
                        //setSaveScreen();
                        //  u8g2.sendBuffer();
                        setMenuScreen();
                     }break;
                     case 2: // MODELLSCREEN
                     {                 
                        
                        setModellScreen();
                     }break;
                     case 3: // FUNKTIONSCREEN
                     {
                        curr_aktion = 0;
                        curr_cursorspalte = 0;
                        setFunktionScreen();
                     }break;

                     case 4: // check
                     {
                        curr_screen = 0;
                        setHomeScreen();

                     }break;
                     case 5: // MODUSSCREEN
                     {

                        setHomeScreen();
                     }break;
                        
                        
                  }// switch curr_screen
               }
               Serial.print("T7 curr_screen: ");
               Serial.println(curr_screen);              
               u8g2.sendBuffer();
               tastaturstatus &=  ~(1<<AKTION_OK);
               tastaturstatus |= (1<<UPDATE_OK); 
               calibstatus &= ~(1<<CALIB_START);   
            }
         }break;
            
         case 8:
         {
            Serial.print("T 8");
            if (tastaturstatus & (1<<AKTION_OK))
            {
               Serial.print("T 8 down");
               tastaturstatus &=  ~(1<<AKTION_OK);
               tastaturstatus |= (1<<UPDATE_OK);
               switch (curr_screen)
               {
                  case 0: // HOMESCREEN
                  {
                     
                  }break;
                  case 1: // MENUSCREEN Modelle
                  {
                     if(curr_model < 5)
                     {
                        curr_model++;
                        updateMenuScreen();
                        u8g2.sendBuffer();
                     }
                  }break;
                     
                  case 2:  // MODELLSCREEN  Funktionen
                  {
                     if(curr_funktion < 4)
                     {
                        curr_funktion++;
                        updateModellScreen();
                        u8g2.sendBuffer();
                     }
                  }break;
                     
                  case 3:  // FUNKTIONSCREEN 
                  {
                     if(curr_aktion < 5)
                     {
                        curr_aktion++;
                        updateFunktionScreen();
                        u8g2.sendBuffer();
                     }
                  }break;
                     
                  case 4: // T8 AKTIONSCREEN
                  {
                     switch (curr_cursorspalte)
                     {
                        case 0:
                        {
                           if(curr_wert <2)
                           {
                              curr_wert++;
                              updateAktionScreen();
                              u8g2.sendBuffer();
                           }
                        }break;
                        case 1: // T8 UP DOWN
                        {
                           uint8_t level = kanalsettingarray[curr_model][curr_funktion][1];
                           uint8_t levelO = (level & 0xF0) >> 4;
                           uint8_t levelU = (level & 0x0F);
                           
                           uint8_t expo = kanalsettingarray[curr_model][curr_funktion][2];
                           uint8_t expoO = (expo & 0xF0) >> 4;
                           uint8_t expoU = (expo & 0x0F);
                           
                           switch (curr_aktion)
                           {
                              case 0: //LEVEL
                              {
                                 switch (curr_wert)
                                 {
                                    case 0: // UP
                                    {
                                       if(levelO)
                                       {
                                          levelO--;
                                          savestatus = CHANGED;;
                                       }
                                    }break;
                                    case 1: // DOWN
                                    {
                                       if (levelU)
                                       {
                                          levelU--;
                                          savestatus = CHANGED;;
                                       }
                                    }break;
                                 } // switch curr_wert
                                 level = (levelO << 4) | levelU;
                                 kanalsettingarray[curr_model][curr_funktion][1] = level;
                                 
                              }break;
                              case 1: //EXPO
                              {
                                 switch (curr_wert)
                                 {
                                    case 0: // UP
                                    {
                                       if(expoO)
                                       {
                                          expoO--;
                                          savestatus = CHANGED;;
                                       }
                                    }break;
                                    case 1: // DOWN
                                    {
                                       if (expoU)
                                       {
                                          expoU--;
                                          savestatus = CHANGED;;
                                       }
                                    }break;
                                 } // switch curr_wert
                                 expo = (expoO << 4) | expoU;
                                 kanalsettingarray[curr_model][curr_funktion][2] = expo;
                                 
                              }break;
                                 
                                 
                                 
                           }// switch curr_aktion
                           
                           
                        }break;
                     }// switch curr_cursorspalte
                     updateAktionScreen();
                     u8g2.sendBuffer();
                  }break;
                     
                  case 5: // T8 DOWN MODUSSCREEN
                  {

                     if(curr_modus < 2)
                     {
                        curr_modus++;
                        switch (curr_modus)
                        {
                           case MODELL:
                           {

                           }break;

                           case SIM:
                           {

                           }break;

                           case CALIB:
                           {
                              updateModusScreen();
                              u8g2.sendBuffer();
                           }break;
                        }// switch curr_modus
                        
                        
                        
                     }
                  }break;
               }// switch (curr_screen)
               
               
               
            }          
         }break;
            
         case 9:
         {
            Serial.print("T 9 SAVE");  
            
            switch (curr_screen)
            {
               case 0:
               {
                  //Serial.print(" savestatus: ");  
                  //Serial.print(savestatus);
                  //Serial.print(" curr_cursorspalte: ");  
                  Serial.print(curr_cursorspalte);
                  switch (savestatus)
                  {
                     case 0: // CHANGED
                     {
                        // write to eeprom
                        eepromwrite();
                        savestatus = CANCEL;
                     }break;
                     
                     case 1: // CANCEL
                     {
                        // do nothing
                        
                        curr_cursorspalte = 0;
                        savestatus = CANCEL;
                     }break;

                    
                     
                     //u8g2.sendBuffer();
                  }   
               }break;
                  
            }// switch curr_screen
            updateHomeScreen();
         }break;
      }//switch (Taste)
      if(Taste)
      {
         //Serial.print("\n");
         Taste = 0;
         tastaturstatus &= ~(1<<TASTE_OK);
      }
   }
  loopcounter0++;

  if(loopcounter0 >= BLINKRATE)
   {
      loopcounter0 = 0;
      loopcounter1++;
      if(loopcounter1 > 128)
      {
         loopcounter1 = 0;
         //tastaturwert = analogRead(TASTATUR_PIN);
        
        //Serial.print("tastaturwert: ");
        //Serial.print(tastaturwert);

        //Serial.print("\t");
        //Serial.print("tastendelaycounter: ");
        //Serial.print(tastendelaycounter);
        
        //tastenfunktion(tastaturwert);
        
        //Serial.println(blinkcounter);
        // Taste = 0;
        //Serial.print(tastaturwert);

        tastaturwert = 0;
        blinkcounter++;
        impulscounter+=16;
        digitalWrite(LOOPLED, ! digitalRead(LOOPLED));
        digitalWrite(PRINTLED, ! digitalRead(PRINTLED));

         // Batterie
       
      
      }// loopcounter1
   } // if loopcount0



   radiocounter++;
   


  //digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(1000);               // wait for a second
  //digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  //delay(1000);               // wait for a second
}