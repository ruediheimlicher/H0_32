#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>

///
/// @mainpage	Robot_32
///
/// @details	PWM for Servos
///
/// @file		Robot_32.ino
/// @brief		Main sketch
///
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
/// @author		Ruedi Heimlicher
/// @date		14.07.2019 20:01
///
/// @copyright	(c) Ruedi Heimlicher, 2019
////// @see		ReadMe.txt for references
///


// Core library for code-sense - IDE-based
// !!! Help: http://bit.ly/2AdU7cu

#include "Arduino.h"
#include <stdint.h>
#include <ADC.h>
#include <ADC_util.h>

#include <SPI.h>
#include "gpio_MCP23S17.h"

//#include <Wire.h> 
//#include <LiquidCrystal_I2C.h>

#include "lcd.h"
#include "analog.h"

#include <EEPROM.h>

//#include <RF24.h>
//#include <RF24Network.h>

// Load Wi-Fi library
//#include <ESP8266WiFi.h>

//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>


ADC *adc = new ADC(); // adc object
// Set parameters

#define TEST 1
// Include application, user and local libraries
// !!! Help http://bit.ly/2CL22Qp


// Define structures and classes


// Define variables and constants
//#define STARTWERT  2840 // Mitte
#define STARTWERT  2080 // Nullpunkt
#define MAXWERT  4096 // Nullpunkt


#define LOOPLED      0 // 

#define TAKT_PIN     4
#define OUT_PIN      2
#define OUT_PIN_INV  1

#define CONTROL_PIN  6

#define CURR_PIN     A4

#define ANZLOKS       4

#define POT_0_PIN    A0
#define POT_1_PIN    A1
#define POT_2_PIN    A2
#define POT_3_PIN    A3

#define SOURCECONTROL    6 // Eingang, HI wenn local
#define LOKSYNC         7



#define OSZI_PULS_A        8
#define OSZI_PULS_B        9 

volatile uint8_t loopstatus = 0;
volatile uint8_t loopcounter = 0;
volatile uint8_t sourcestatus = 1; // local/USB
#define LOCAL  0
#define USB 1
#define FIRSTRUN  1

#define SERIAL_OK 2

byte buffer[64];

byte sendbuffer[64];


elapsedMillis msUntilNextSend;
unsigned int packetCount = 0;

volatile uint8_t usbtask = 0;

volatile uint8_t teensytask = 0;

volatile uint16_t aktualcommand = 0;

volatile uint16_t commandarray0[26] = {0};

volatile uint16_t taskarray[8][32] = {0};

volatile uint16_t adressearray[4];
volatile uint16_t eepromadressearray[8][4];
volatile uint16_t speedarray[5];

volatile uint8_t loknummer = 0;

volatile uint8_t speed = 0;

volatile uint8_t richtungcounter = 0; // mehrere richtungdatenpakete bei Richtungswechsel

volatile uint8_t richtungstatus = 0;
#define MAXRICHTUNGCOUNTER 4 // max anzahl richtungdatenpakete
#define RICHTUNGSTART 0
#define RICHTUNGEND 1


uint8_t minanzeige = 0xFF;
//let GET_U:UInt8 = 0xA2
//let GET_I:UInt8 = 0xB2

// sinus
elapsedMillis sinms;
elapsedMillis sinceblink;


float sinpos = 0;
#define pi 3.14
#define SIN_START   0xE0
#define SIN_STOP   0xE1

#define SPI_CLK   13
#define SPI_MISO  12
#define SPI_MOSI  11
#define SPI_CS    10



#define ANZLOKALLOKS       2 // anz loks bei lokalem Betrieb


gpio_MCP23S17 mcp0(10,0x20);//instance 0 (address A0,A1,A2 tied to 0)
uint8_t regA = 0x0;
uint8_t regB = 0;

volatile uint8_t tastencodeA = 0;
volatile uint8_t tastencodeB = 0;
uint8_t tastenstatusA = 0;
//pi.__BEGIN_DECLS

volatile uint8_t tastenadresseA = 0;
volatile uint8_t tastenadresseB = 0;

volatile uint8_t lokaladressearray[ANZLOKALLOKS] = {}; // Lok-Adressen
volatile uint8_t lokalcodearray[ANZLOKALLOKS] = {}; // Lok-Codes (Richtung, Funktion)

uint8_t tastenstatusB = 0;

uint8_t potarray[ANZLOKS] = {};
uint8_t localpotarray[ANZLOKALLOKS] = {};


uint8_t lokalstatus = 0;
#define LOKALRICHTUNGBIT0 0
#define LOKALRICHTUNGBIT1 1
#define LOKALRICHTUNGBIT2 2
#define LOKALRICHTUNGBIT3 3
elapsedMillis sincelocalrichtung;

uint8_t potpinarray[4] = {POT_0_PIN,POT_1_PIN,POT_2_PIN,POT_3_PIN};

char* buffercode[4] = {"BUFFER_FAIL","BUFFER_SUCCESS", "BUFFER_FULL", "BUFFER_EMPTY"};

// Prototypes
// !!! Help: http://bit.ly/2l0ZhTa

#define HI     0xFEFE  // 1111111011111110
#define LO     0x0202  // 0000001000000010
#define OPEN   0x02FE  // 0000001011111110

#define TIMERINTERVALL  26
#define PAUSE 10
// Utilities
elapsedMillis sinceringbuffer;

elapsedMillis sincewegbuffer;

elapsedMillis sinceemitter;

elapsedMillis sincemcp;

uint16_t abschnittindex = 0; // aktuelles Element in positionsarray

// Create an IntervalTimer object 
IntervalTimer              paketTimer;
volatile uint16_t          timerintervall = TIMERINTERVALL;

IntervalTimer              stromTimer;
volatile uint16_t          emitter = 0;
volatile uint16_t          emitterarray[8] = {0};
volatile uint16_t          emittermittel = 0;
volatile uint8_t           emittermittelcounter = 0;
volatile uint8_t           emitterNULL = 330;
volatile uint8_t           pause = PAUSE;
volatile uint8_t           richtung = 1; // vorwaerts

volatile uint8_t           paketpos = 0;
volatile uint8_t           paketmax = ANZLOKS;

volatile uint8_t           commandpos = 0; // pos im command
volatile uint8_t           bytepos = 0; // pos im Ablauf

uint16_t                   tritarray[] = {LO,OPEN,HI};

int achse0_startwert=0;

//LiquidCrystal_I2C lcd(0x27,20,4); 

void printHex8(uint8_t data) // prints 8-bit data in hex with leading zeroes
{
   Serial.print("0x"); 
  // for (int i=0; i<length; i++) 
   { 
      if (data<0x10) 
      {
         Serial.print("0");
         
      } 
      Serial.print(data,HEX); 
      Serial.println(" "); 
   }
}
// Functions

void OSZI_A_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_A,LOW);
}

void OSZI_A_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_A,HIGH);
}

void OSZI_A_TOGG(void)
{
   if (TEST)
      digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
}

void OSZI_B_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_B,LOW);
}

void OSZI_B_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_B,HIGH);
}

void OSZI_B_TOGG(void)
{
   if (TEST)
      digitalWrite(OSZI_PULS_B, !digitalRead(OSZI_PULS_B));
}

void pakettimerfunction() 
{ 
   /*
    commands
    LO     0x0202  // 0000001000000010
    OPEN   0x02FE  // 0000001011111110
    HI     0xFEFE  // 1111111011111110
    */
   
   digitalWriteFast(TAKT_PIN, !digitalReadFast(TAKT_PIN)); // toggle
   
   aktualcommand = taskarray[paketpos][bytepos]; // zu schickendes command
   
   
   if ((bytepos) == 0)
   {
      
      if(paketpos == 0)
         {
         //OSZI_B_LO();
         }
      
      if ((sourcestatus & 0x01) && (paketpos == 0))
      {
         OSZI_A_LO();
         digitalWriteFast(LOKSYNC,LOW);
      }
      //
      else if ((sourcestatus & 0x02) && (paketpos == loknummer))
      {
         digitalWriteFast(LOKSYNC,LOW);
      }
   }
   
   if (aktualcommand & (1<<commandpos))
   {
      digitalWriteFast(OUT_PIN,HIGH);
      digitalWriteFast(OUT_PIN_INV,LOW);
      
      digitalWriteFast(CONTROL_PIN,HIGH);
      
   }
   else
   {
      digitalWriteFast(OUT_PIN,LOW);
      digitalWriteFast(OUT_PIN_INV,HIGH);
      
      digitalWriteFast(CONTROL_PIN,LOW);
   }
   if (commandpos < 15)
   {
      commandpos++;
   }
   else 
   {
      commandpos = 0;
      digitalWriteFast(LOKSYNC,HIGH);
      
      digitalWriteFast(OUT_PIN,LOW);
      digitalWriteFast(OUT_PIN_INV,HIGH);
      
      bytepos++;
      if (bytepos >= 20 + pause) // Paket fertig
      {
         bytepos = 0;
         OSZI_A_HI();
         OSZI_B_LO();
         if (paketpos < paketmax - 1)
         {
            paketpos++; // jede Lok ein Paket
            if (paketpos == paketmax - 1) // Paket fertig
            {
               //
               OSZI_B_HI();
               paketpos = 0;
            }
         }
         else 
         {
            
            //paketpos = 0;
         }
         
      }
   
   }
   
   
   //taskarray[0][5] = LO;
   
}

void LCD_init(void)
{
   pinMode(LCD_RSDS_PIN, OUTPUT);
   pinMode(LCD_ENABLE_PIN, OUTPUT);
   pinMode(LCD_CLOCK_PIN, OUTPUT);
   digitalWrite(LCD_RSDS_PIN,1);
   digitalWrite(LCD_ENABLE_PIN,1);
   digitalWrite(LCD_CLOCK_PIN,1);

}

void ADC_init(void) 
{
   emitter=0; // 
   
   adc->adc0->setAveraging(4); // set number of averages 
   adc->adc0->setResolution(8); // set bits of resolution
   adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);
   adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
   adc->adc0->setReference(ADC_REFERENCE::REF_3V3);
  // adc->adc0->enableInterrupts(ADC_0);
   
   
//   delay(100);
   
   
}


void stromtimerfunction()
{
   emitter = adc->analogRead(CURR_PIN);
   emitterarray[emittermittelcounter & 0x07] = emitter;
   emittermittelcounter++;
   
}


// Add setup code
void setup()
{
   Serial.begin(9600);
 //while (!Serial) ;
   loopstatus |= (1<<FIRSTRUN); // Bit fuer tasks in erster Runde
   delay(100);
 //  analogWriteResolution(16); // 32767
   
   paketTimer.begin(pakettimerfunction,timerintervall);
   paketTimer.priority(0);
   
   //stromTimer.begin(stromtimerfunction, 5000);
   
   pinMode(LOOPLED, OUTPUT);
   
   // FTM0   Pins: 5, 6, 9, 10, 20, 21, 22, 23
   // FTM1   3, 4   
   // FTM2   25, 32
   analogWriteFrequency(5, 50);
   Serial.println(F("RawHID H0"));
 
   pinMode(TAKT_PIN, OUTPUT);
   digitalWriteFast(TAKT_PIN, LOW); // LO, OFF 

   pinMode(OUT_PIN, OUTPUT);
   digitalWriteFast(OUT_PIN, LOW); // LO, OFF 

   // Signal Invertiert
   pinMode(OUT_PIN_INV, OUTPUT);
    digitalWriteFast(OUT_PIN_INV, HIGH); // HI, OFF 

   // Control
   pinMode(CONTROL_PIN, OUTPUT);
   digitalWriteFast(CONTROL_PIN, HIGH); // HI, OFF 

   
   pinMode(LOKSYNC, OUTPUT);
   digitalWriteFast(LOKSYNC, HIGH); 
   
   pinMode(OSZI_PULS_A, OUTPUT);
   digitalWriteFast(OSZI_PULS_A, HIGH); 
   pinMode(OSZI_PULS_B, OUTPUT);
   digitalWriteFast(OSZI_PULS_B, HIGH); 
   
   //ghpinMode(SOURCECONTROL, INPUT);
   
   LCD_init();
   
   
   mcp0.begin();
   /*
    • PortA registeraddresses range from 00h–0Ah
    • PortB registeraddresses range from 10h–1Ah
    PortA output, PortB input: Direction 1 output: direction 0
    0x0F: A: out B: in
    */

   //mcp0.gpioPinMode(0x00FF); // A Ausgang, B Eingang
   mcp0.gpioPinMode(0xFFFF); // alle input
   
   //mcp0.portPullup(0x00FF); 
   mcp0.portPullup(0xFFFF);// alle HI
   
   mcp0.gpioPort(0xCC33);

   
   EEPROM.begin();
    
   delay(100);
   usbtask = 0;
   adressearray[0] = LO;
   adressearray[1] = HI;
   adressearray[2] = LO;
   adressearray[3] = OPEN;
   
   speed = 0;
   Serial.print("speed: ");
   Serial.print(speed);
   Serial.print("\n");
   Serial.print("a: ");
   Serial.print(speed & (1<<0));
   Serial.print(" b: ");
   Serial.print(speed & (1<<1));
   Serial.print(" c: ");
   Serial.print(speed & (1<<2));
   Serial.print(" d: ");
   Serial.print(speed & (1<<3));
   Serial.print("\n");

  // init_analog();
   for (uint8_t i=0;i<4;i++)
   {
      Serial.print(" i: "); Serial.print(i);
      Serial.print(" data: ");Serial.print(speed & (1<<i));
      Serial.print("\n");
      if (speed & (1<<i))
      {
         Serial.print("HI");
         speedarray[i] = HI; 
      }
      else
      {
         Serial.print("LO");
         speedarray[i] = LO; 
      }
      Serial.print("\n");
   }
 
   eepromadressearray[0][0] = tritarray[buffer[8]];
   
   taskarray[0][0] = adressearray[0];
   taskarray[0][1] = adressearray[1];
   taskarray[0][2] = adressearray[2];
   taskarray[0][3] = adressearray[3];
   taskarray[0][4] = HI; // Lampe
   taskarray[0][5] = speedarray[0];
   taskarray[0][6] = speedarray[1];
   taskarray[0][7] = speedarray[2];
   taskarray[0][8] = speedarray[3];

   // pause
   taskarray[0][9] = 0;
   taskarray[0][10] = 0;
   taskarray[0][11] = 0;
   
   // wiederholung
   taskarray[0][12] = taskarray[0][0];
   taskarray[0][13] = taskarray[0][1];
   taskarray[0][14] = taskarray[0][2];
   taskarray[0][15] = taskarray[0][3];
   taskarray[0][16] = taskarray[0][4];
   taskarray[0][17] = taskarray[0][5];
   taskarray[0][18] = taskarray[0][6];
   taskarray[0][19] = taskarray[0][7];
   taskarray[0][20] = taskarray[0][8];

   
 // Paket 1
   adressearray[0] = OPEN;
   adressearray[1] = OPEN;
   adressearray[2] = OPEN;
   adressearray[3] = OPEN;

   taskarray[1][0] = adressearray[0];
   taskarray[1][1] = adressearray[1];
   taskarray[1][2] = adressearray[2];
   taskarray[1][3] = adressearray[3];
   taskarray[1][4] = HI; // Lampe
   taskarray[1][5] = speedarray[0];
   taskarray[1][6] = speedarray[1];
   taskarray[1][7] = speedarray[2];
   taskarray[1][8] = speedarray[3];
   
   // pause
   taskarray[1][9] = 0;
   taskarray[1][10] = 0;
   taskarray[1][11] = 0;
   
   // wiederholung
   taskarray[1][12] = taskarray[1][0];
   taskarray[1][13] = taskarray[1][1];
   taskarray[1][14] = taskarray[1][2];
   taskarray[1][15] = taskarray[1][3];
   taskarray[1][16] = taskarray[1][4];
   taskarray[1][17] = taskarray[1][5];
   taskarray[1][18] = taskarray[1][6];
   taskarray[1][19] = taskarray[1][7];
   taskarray[1][20] = taskarray[1][8];
   

   // paket 2
   
   taskarray[2][0] = adressearray[0];
   taskarray[2][1] = adressearray[1];
   taskarray[2][2] = adressearray[2];
   taskarray[2][3] = adressearray[3];
   taskarray[2][4] = HI; // Lampe
   taskarray[2][5] = speedarray[0];
   taskarray[2][6] = speedarray[1];
   taskarray[2][7] = speedarray[2];
   taskarray[2][8] = speedarray[3];
   
   // pause
   taskarray[2][9] = 0;
   taskarray[2][10] = 0;
   taskarray[2][11] = 0;
   
   // wiederholung
   taskarray[2][12] = taskarray[2][0];
   taskarray[2][13] = taskarray[2][1];
   taskarray[2][14] = taskarray[2][2];
   taskarray[2][15] = taskarray[2][3];
   taskarray[2][16] = taskarray[2][4];
   taskarray[2][17] = taskarray[2][5];
   taskarray[2][18] = taskarray[2][6];
   taskarray[2][19] = taskarray[2][7];
   taskarray[2][20] = taskarray[2][8];

   /*
   commandarray0[0] = adressearray[0];
   commandarray0[1] = adressearray[1];
   commandarray0[2] = adressearray[2];
   commandarray0[3] = adressearray[3];
   commandarray0[4] = HI; // Lampe
   commandarray0[5] = speedarray[0];
   commandarray0[6] = speedarray[1];
   commandarray0[7] = speedarray[2];
   commandarray0[8] = speedarray[3];
   
   commandarray0[9] = 0;
   commandarray0[10] = 0;
   commandarray0[11] = 0;
   
   commandarray0[12] = commandarray0[0];
   commandarray0[13] = commandarray0[1];
   commandarray0[14] = commandarray0[2];
   commandarray0[15] = commandarray0[3];
   commandarray0[16] = commandarray0[4];
   commandarray0[17] = commandarray0[5];
   commandarray0[18] = commandarray0[6];
   commandarray0[19] = commandarray0[7];
   commandarray0[20] = commandarray0[8];
   */
   aktualcommand = OPEN;
   
   pinMode(POT_0_PIN, INPUT);
   pinMode(POT_1_PIN, INPUT);
   pinMode(POT_2_PIN, INPUT);
   pinMode(CURR_PIN, INPUT);
   
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   _delay_ms(100);
   lcd_puts("Teensy");

   
   ADC_init();
   delay(100);
   Serial.print("setup: ");
   //  lcd.init();
    //  lcd.backlight();
    //  lcd.setCursor(0,0);
    //  lcd.print("H0-32");
    _delay_ms(200);
    //  lcd.clear();  
   uint8_t eepromtimerintervall = EEPROM.read(0xA0);
   if (eepromtimerintervall < 0xFF) // schon ein Wert gespeichert
   {
      timerintervall = eepromtimerintervall;
   }
    //  lcd.setCursor(0,0);
   //  lcd.print(timerintervall);
    uint8_t eeprompos = 0x00;
    uint8_t eepromadressbyte = 0;
    /*
    eepromadressbyte = EEPROM.read(eeprompos++);
    Serial.print("adresse: ");
    Serial.print(eeprompos);
    Serial.print(" byte: ");
    Serial.println(eepromadressbyte);
     Serial.print("\n");
*/
  
    taskarray[0][1] = tritarray[buffer[9]];
    taskarray[0][2] = tritarray[buffer[10]];
    taskarray[0][3] = tritarray[buffer[11]];

   
   //  lcd.setCursor(0,3);;
    
/*
   delay(50);
      eepromadressbyte = EEPROM.read(eeprompos);
   taskarray[0][0] = tritarray[eepromadressbyte];
      Serial.print("adresse: ");
      Serial.print(eeprompos);
      Serial.print(" byte: ");
      Serial.println(eepromadressbyte);
   //  lcd.print(eepromadressbyte);
     delay(50);
     eeprompos++;
     eepromadressbyte = EEPROM.read(eeprompos);
   taskarray[0][1] = tritarray[eepromadressbyte];
         Serial.print("adresse: ");
         Serial.print(eeprompos);
         Serial.print(" byte: ");
         Serial.println(eepromadressbyte);
   //  lcd.print(eepromadressbyte);
     delay(10);
     eeprompos++;
     eepromadressbyte = EEPROM.read(eeprompos);
   taskarray[0][2] = tritarray[eepromadressbyte];
         Serial.print("adresse: ");
         Serial.print(eeprompos);
         Serial.print(" byte: ");
         Serial.println(eepromadressbyte);
   //  lcd.print(eepromadressbyte);
     delay(10);
     eeprompos++;
     eepromadressbyte = EEPROM.read(eeprompos);
     taskarray[0][3] = tritarray[eepromadressbyte];
         Serial.print("adresse: ");
         Serial.print(eeprompos);
         Serial.print(" byte: ");
         Serial.println(eepromadressbyte);
   //  lcd.print(eepromadressbyte);

   // repetition address
     taskarray[0][12] = taskarray[0][0] ;
     taskarray[0][13] = taskarray[0][1] ;
     taskarray[0][14] = taskarray[0][2] ;
     taskarray[0][15] = taskarray[0][3] ;
   // pause
    taskarray[0][9] = 0;
    taskarray[0][10] = 0;
    taskarray[0][11] = 0;

   delay(50);
   
   eeprompos = 0x08;
       eepromadressbyte = EEPROM.read(eeprompos);
    taskarray[1][0] = tritarray[eepromadressbyte];
       Serial.print("adresse: ");
       Serial.print(eeprompos);
       Serial.print(" byte: ");
       Serial.println(eepromadressbyte);
    //  lcd.print(" ");
   //  lcd.print(eepromadressbyte);

      delay(50);
      eeprompos++;
      eepromadressbyte = EEPROM.read(eeprompos);
    taskarray[1][1] = tritarray[eepromadressbyte];
          Serial.print("adresse: ");
          Serial.print(eeprompos);
          Serial.print(" byte: ");
          Serial.println(eepromadressbyte);
   //  lcd.print(eepromadressbyte);
      delay(10);
      eeprompos++;
      eepromadressbyte = EEPROM.read(eeprompos);
    taskarray[1][2] = tritarray[eepromadressbyte];
          Serial.print("adresse: ");
          Serial.print(eeprompos);
          Serial.print(" byte: ");
          Serial.println(eepromadressbyte);
   //  lcd.print(eepromadressbyte);
      delay(10);
      eeprompos++;
      eepromadressbyte = EEPROM.read(eeprompos);
      taskarray[1][3] = tritarray[eepromadressbyte];
          Serial.print("adresse: ");
          Serial.print(eeprompos);
          Serial.print(" byte: ");
          Serial.println(eepromadressbyte);
//  lcd.print(eepromadressbyte);
   
   delay(50);
   
    eeprompos = 0x10;
        eepromadressbyte = EEPROM.read(eeprompos);
     taskarray[2][0] = tritarray[eepromadressbyte];
        Serial.print("adresse: ");
        Serial.print(eeprompos);
        Serial.print(" byte: ");
        Serial.println(eepromadressbyte);
    //  lcd.print(" ");
   //  lcd.print(eepromadressbyte);

       delay(50);
       eeprompos++;
       eepromadressbyte = EEPROM.read(eeprompos);
     taskarray[2][1] = tritarray[eepromadressbyte];
           Serial.print("adresse: ");
           Serial.print(eeprompos);
           Serial.print(" byte: ");
           Serial.println(eepromadressbyte);
   //  lcd.print(eepromadressbyte);
       delay(10);
       eeprompos++;
       eepromadressbyte = EEPROM.read(eeprompos);
     taskarray[2][2] = tritarray[eepromadressbyte];
           Serial.print("adresse: ");
           Serial.print(eeprompos);
           Serial.print(" byte: ");
           Serial.println(eepromadressbyte);
   //  lcd.print(eepromadressbyte);
       delay(10);
       eeprompos++;
       eepromadressbyte = EEPROM.read(eeprompos);
       taskarray[2][3] = tritarray[eepromadressbyte];
   
           Serial.print("adresse: ");
           Serial.print(eeprompos);
           Serial.print(" byte: ");
           Serial.println(eepromadressbyte);
//  lcd.print(eepromadressbyte);
 */ 
   
   byte null[8] = 
   {
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000   };  
    lcd_setcustom(0,null);
     //  lcd.createChar(0, smiley);
   byte eins[8] = {
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b11111
   };
   //  lcd.createChar(1, eins);
   lcd_setcustom(1,eins);
   byte zwei[8] = {
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b11111,
      0b11111
   };
   //  lcd.createChar(2, zwei);
   lcd_setcustom(2,zwei);
   byte drei[8] = {
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b11111,
      0b11111,
      0b11111
   };
   //  lcd.createChar(3, drei);
   lcd_setcustom(3,drei);
   byte vier[8] = {
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b11111,
      0b11111,
      0b11111,
      0b11111
   };
   //  lcd.createChar(4, vier);
   lcd_setcustom(4,vier);
   byte fuenf[8] = {
      0b00000,
      0b00000,
      0b00000,
      0b11111,
      0b11111,
      0b11111,
      0b11111,
      0b11111
   };
   //  lcd.createChar(5, fuenf);
   lcd_setcustom(5,fuenf);
   byte sechs[8] = {
      0b00000,
      0b00000,
      0b11111,
      0b11111,
      0b11111,
      0b11111,
      0b11111,
      0b11111
   };
   //  lcd.createChar(6, sechs);
   lcd_setcustom(6,sechs);
   byte sieben[8] = {
      0b00000,
      0b11111,
      0b11111,
      0b11111,
      0b11111,
      0b11111,
      0b11111,
      0b11111
   };
   //  lcd.createChar(7, sieben);
   lcd_setcustom(7,sieben);


}

// Add loop code
void loop()
{
#pragma mark mcp
   if (sincemcp > 10)
   {
      /*
      if (Serial)
      {
         loopstatus |= (1<<SERIAL_OK);
      }
      else
      {
//        loopstatus &= ~(1<<SERIAL_OK);
      }
       */
      
      sincemcp = 0;
      // bit 0: Funktion
      // bit 1: Richtungsimpuls
      
      // bit 4-7: Adresse lesen: SPI MCP23S17
      tastencodeA = 0xFF - mcp0.gpioReadPortA(); // active taste ist LO > invertieren
      tastenadresseA = (tastencodeA & 0xF0) >> 4; // Bit 7-4
      lokaladressearray[0] = (tastencodeA & 0xF0) >> 4;
      
      
      //lokaladressearray[0] ^= lokaladressearray[0];
      lokalcodearray[0] = tastencodeA & 0x0F; // Bit 0-3
      
      /*
      for (uint8_t i=0;i<4;i++)
      {
         if (tastenadresseA & (1<<i))
         {
            taskarray[0][i] = HI;
         }
         else
         {
            taskarray[0][i] = LO;
         }
      }
      */
      tastencodeB = 0xFF - mcp0.gpioReadPortB(); // active taste ist LO > invertieren
      tastenadresseB = (tastencodeB & 0xF0) >> 4;
      
      lokaladressearray[1] = (tastencodeB & 0xF0) >> 4;
      lokalcodearray[1] = tastencodeB & 0x0F;// Bit 0-3
/*
      for (uint8_t i=0;i<4;i++)
       {
          if (tastenadresseB & (1<<i))
          {
             taskarray[1][i] = HI;
          }
          else
          {
             taskarray[1][i] = LO;
          }
       }
*/
      
      tastenstatusA |= tastencodeB;
      
      
      // Pot auslesen
      for (uint8_t i=0;i<ANZLOKALLOKS;i++)
      {
         localpotarray[i] = adc->analogRead(potpinarray[i]); // 8 bit
         sendbuffer[16+i] = localpotarray[i];
      }
      
   } // if (sincemcp )
   
#pragma mark EMITTER 
   if (sinceemitter > 200)
   {
      sinceemitter = 0;
      //     emitter = adc->analogRead(CURR_PIN); // in stromtimerfunktion
      
      //Serial.print(" data: \t");
      emittermittel = 0;
      for (uint8_t i=0;i<8;i++)
      {
         
         //       Serial.print(emitterarray[i]);
         //       Serial.print("\t");
         emittermittel += emitterarray[i];
      }
      
      //    Serial.print("\t");
      emittermittel /= 8;
 //     Serial.print("emittermittel: ");
 //     Serial.print(emittermittel);
      // Serial.print(byte(0));
      
  //     Serial.print("\n");
      
      //  lcd.setCursor(4,0);
      if (emitter < 0xFF)
      {
         
         uint8_t anzeige = (emittermittel);
         anzeige = anzeige ^ 0xFF;
         if (anzeige < minanzeige)
         {
            minanzeige = anzeige;
         }
         lcd_gotoxy(0, 3);
         
         lcd_putint(0xFF - emittermittel);
         lcd_putc(' ');
         lcd_putint(emittermittel);
         lcd_putc(' ');
      }
      else 
      {
         //  lcd.print("         ");
      }
      ////  lcd.setCursor(18,0);
      uint8_t pos = (emittermittel)/10;
       
      ////  lcd.print("*");
      sendbuffer[10] = 0xAB;
      sendbuffer[12] = emitter & 0x00FF;
      sendbuffer[13] = (emitter & 0xFF00)>>8;
      /*
       uint8_t n = RawHID.send(sendbuffer, 10);
       if (n > 0) 
       {
       Serial.print(F("Transmit packet "));
       Serial.println(n);
       // Serial.print(" count: ");
       // Serial.println(packetCount );
       packetCount = packetCount + 1;
       } else 
       {
       Serial.println(F("Unable to transmit packet"));
       }
       */
      //    Serial.print("emittermittel: ");
      //    Serial.print(emittermittel);
      //    Serial.print("\n");
      
      /*  
       uint16_t pot0 = readPot(A0);
       //  lcd.setCursor(12,0);
       if (pot0 < 10)
       {
       //  lcd.print("  ");
       }
       else if (pot0 < 100)
       {
       //  lcd.print(" ");
       }
       //  lcd.setCursor(12,0);
       //  lcd.print(pot0);
       */
   } // if sinceemitter
   
#pragma mark blink 
   if (sinceblink > 500)
   {
      sinceblink = 0;
      loopcounter++;
      //_delay_ms(10);
      //pinMode(LOOPLED, OUTPUT);
      digitalWriteFast(LOOPLED, !digitalReadFast(LOOPLED));
      //lcd_putc('a');
      //lcd_puts("blink");
      lcd_gotoxy(16, 0);
      lcd_puthex(sourcestatus);
      lcd_gotoxy(0,1);
      lcd_puts("A ");
      lcd_puthex(tastencodeA);
      lcd_putc(' ');
      lcd_puts("B ");
      lcd_puthex(tastencodeB);
      lcd_putc(' ');
      lcd_puthex(loopstatus);
      lcd_putc(' ');
      lcd_putint(speed);
      uint8_t strompos = (emittermittel)/10;
      lcd_gotoxy(18,3);
      lcd_putc(loopcounter & 0x07);
      lcd_putc(strompos);
      lcd_gotoxy(0, 2);
      lcd_putint(localpotarray[0]);
      lcd_putc(' ');
      lcd_puthex(taskarray[0]);
      lcd_putc(' ');
      lcd_putint(localpotarray[1]);
      lcd_putc(' ');
      lcd_puthex(taskarray[1]);
      //Serial.print("speed: ");
      //Serial.print(speed);
      //Serial.printf("USB sourcestatus 2: %d\n ",sourcestatus);
      /*
       Serial.print("speed: ");
       Serial.print(speed);
       Serial.print("\n");
       Serial.print(" a: ");
       Serial.print(speed & (1<<0));
       Serial.print(" b: ");
       Serial.print(speed & (1<<1));
       Serial.print(" c: ");
       Serial.print(speed & (1<<2));
       Serial.print(" d: ");
       Serial.print(speed & (1<<3));
       Serial.print("\n");
       
       if (speed > 15)
       {
       speed = 0;
       }
       */
      /*
       for (uint8_t i=0;i<4;i++)
       {
       //   Serial.print(" i: ");
       //   Serial.print(i);
       //   Serial.print(" data: ");
       //   Serial.print(speed & (1<<i));
       
       if ((speed & (1<<i)) > 0)
       {
       //            Serial.print(" HI");
       commandarray0[i+5] = HI; 
       commandarray0[i+17] = HI; 
       }
       else
       {
       //            Serial.print(" LO");
       commandarray0[i+5] = LO; 
       commandarray0[i+17] = LO; 
       }
       //         Serial.print("\n");
       }
       */
      /*
       Serial.print("speed: ");
       Serial.print(" ");
       Serial.print(commandarray0[5]);
       Serial.print(" ");
       Serial.print(commandarray0[6]);
       Serial.print(" ");
       Serial.print(commandarray0[7]);
       Serial.print(" ");
       Serial.print(commandarray0[8]);
       Serial.print("\n");
       */
      //      speed++;
      
      /*
       uint8_t erfolg = ringbufferIn(erstepos);
       
       uint16_t anz = ringbufferCount();
       Serial.print("erfolg: ");
       Serial.print(erfolg);
       Serial.print(" anzahl: ");
       Serial.print(anz);
       Serial.print(" read: ");
       Serial.print(ringbuffer.read);
       Serial.print(" write: ");
       Serial.println(ringbuffer.write);
       */
      //Serial.print("\n");
   } // if sincblinkk 500
   
   #pragma mark USB
   int n;
   n = RawHID.recv(buffer, 10); // 
   if (n > 0) 
   {
      // the computer sent a message.  Display the bits
      // of the first byte on pin 0 to 7.  Ignore the
      // other 63 bytes!
      //Serial.print(F("Received packet, erstes byte: "));
      //Serial.println((int)buffer[0]);
      //     for (int i=0; i<8; i++) 
      {
         //       int b = buffer[0] & (1 << i);
         //       Serial.print((int)buffer[i]);
         //       Serial.print("\t");
         //       digitalWrite(i, b);
      }
      //     Serial.println();
      //     Serial.print(hb);
      //     Serial.print("\t");
      //     Serial.print(lb);
      //     Serial.println();
      
      // ************************************
      usbtask = buffer[0]; // Auswahl lok
      // ************************************
      loknummer = buffer[20];
      
      sourcestatus = buffer[21];
      
      //Serial.println(" ");
      Serial.print("******************  usbtask *** ");
      printHex8(usbtask);
      Serial.printf("USB sourcestatus: %d loknummer: %d\n",sourcestatus,loknummer);
      for (int i=0; i<24; i++) 
      {
         Serial.print(buffer[i]);
         Serial.print(" ");
      }
      Serial.print("\n");
      
      if (timerintervall != buffer[18])
      {
        // Serial.print("timerintervall changed\n");
         
         paketTimer.update(buffer[18]); 
         timerintervall = buffer[18];
      //   Serial.print(timerintervall);
         EEPROM.update(0xA0,timerintervall);
         
         if ( loopstatus & (1<<FIRSTRUN))
         {
            
         }
      }
      
     
      Serial.printf("USB sourcestatus 2: %d\n ",sourcestatus);
#pragma mark TASK 
      if (sourcestatus & 0x02)
      {
         switch (usbtask)
         {
            case 0xA0: // address
            {
               /*
                uint8_t eeprompos = 0;
                uint8_t eepromadressbyte = 0;
                
                eepromadressbyte = EEPROM.read(eeprompos);
                Serial.print("\nadresse: ");
                Serial.print(eeprompos);
                Serial.print(" byte: ");
                Serial.println(eepromadressbyte);
                
                eeprompos++;
                eepromadressbyte = EEPROM.read(eeprompos);
                Serial.print("adresse: ");
                Serial.print(eeprompos);
                Serial.print(" byte: ");
                Serial.println(eepromadressbyte);
                
                eeprompos++;
                eepromadressbyte = EEPROM.read(eeprompos);
                Serial.print("adresse: ");
                Serial.print(eeprompos);
                Serial.print(" byte: ");
                Serial.println(eepromadressbyte);
                
                eeprompos++;
                eepromadressbyte = EEPROM.read(eeprompos);
                
                Serial.print("adresse: ");
                Serial.print(eeprompos);
                Serial.print(" byte: ");
                Serial.println(eepromadressbyte);
                
                */
               
               taskarray[0][0] = tritarray[buffer[8]];
               taskarray[0][1] = tritarray[buffer[9]];
               taskarray[0][2] = tritarray[buffer[10]];
               taskarray[0][3] = tritarray[buffer[11]];
               
               uint8_t eeprompos = 0;
               
               eeprompos = 0;
               eepromadressearray[0][0] = tritarray[buffer[8]];
               //        EEPROM.update(eeprompos++,buffer[8]);
               delay(10);
               eepromadressearray[0][1] = tritarray[buffer[9]];
               //        EEPROM.update(eeprompos++,buffer[9]);
               delay(10);
               eepromadressearray[0][2] = tritarray[buffer[10]];
               //        EEPROM.update(eeprompos++,buffer[10]);
               delay(10);
               eepromadressearray[0][3] = tritarray[buffer[11]];
               //        EEPROM.update(eeprompos,buffer[11]);
               delay(10);
               
               
               // repetition
               taskarray[0][12] = taskarray[0][0] ;
               taskarray[0][13] = taskarray[0][1] ;
               taskarray[0][14] = taskarray[0][2] ;
               taskarray[0][15] = taskarray[0][3] ;
               
               Serial.print(" usb adress: ");
               loknummer =0;
               Serial.print(eepromadressearray[loknummer][0],HEX);
               Serial.print(" ");
               Serial.print(eepromadressearray[loknummer][1],HEX);
               Serial.print(" ");
               Serial.print(eepromadressearray[loknummer][2],HEX);
               Serial.print(" ");
               Serial.print(eepromadressearray[loknummer][3],HEX);
               Serial.print("\n");
               
               // EEPROM.update(0,eepromadressearray);  
               
            }break;
               
            case 0xB0: // speed
            {
               
               // Adresse mitgeben
               taskarray[0][0] = tritarray[buffer[8]];
               taskarray[0][1] = tritarray[buffer[9]];
               taskarray[0][2] = tritarray[buffer[10]];
               taskarray[0][3] = tritarray[buffer[11]];
               
               eepromadressearray[0][0] = tritarray[buffer[8]];
               eepromadressearray[0][1] = tritarray[buffer[9]];
               eepromadressearray[0][2] = tritarray[buffer[10]];
               eepromadressearray[0][3] = tritarray[buffer[11]];
               
               // repetition address
               taskarray[0][12] = taskarray[0][0] ;
               taskarray[0][13] = taskarray[0][1] ;
               taskarray[0][14] = taskarray[0][2] ;
               taskarray[0][15] = taskarray[0][3] ;
               
               //Serial.print("usbtaskask 0xB0");
               //     taskarray[0][4] = tritarray[(buffer[16] & 0x01)]; // Licht, bit 0
               
               uint8_t speed_raw = buffer[17]; // 0: halt 1: richtung 2-5: speed
               uint8_t speed_red = 0;
               //             Serial.print("speed_raw 0: ");
               //             Serial.println(speed_raw);
   
               if (speed_raw < 10)
               {
                  //  lcd.print(" ");
               }
               else
               {
                  // //  lcd.print("speed ");
               }
               //  lcd.print(speed_raw);
               
               
               if (speed_raw < 2) // stillstand oder Richtungswachsel
               {
                  
                  for (uint8_t i=0;i<4;i++)
                  {
                     Serial.println("speed_raw 0: HALT");
                     //Serial.println(speed_raw);
                     
                     taskarray[0][5+i] = LO;
                  }
                  if (speed_raw == 1)
                  {
                     Serial.println("speed_raw 0: WENDEN");
                     taskarray[0][5] = HI; // richtungswechsel fuer speed = 1
                     
                     
                  }
               }
               else 
               {
                  speed = speed_raw;
                  
                  Serial.print("speed 0: ");
                  Serial.println(speed);
                  for (uint8_t i=0;i<4;i++)
                  {
                     Serial.print(" i: "); Serial.print(i);
                     Serial.print(" data: ");Serial.print(speed & (1<<i));
                     Serial.print("\n");
                     if (speed & (1<<i))
                     {
                        Serial.print("HI");
                        speedarray[i] = HI; 
                        //taskarray[0][8-i] = HI;
                        taskarray[0][5+i] = HI;
                     }
                     else
                     {
                        Serial.print("LO");
                        speedarray[i] = LO; 
                        //taskarray[0][8-i] = LO;
                        taskarray[0][5+i] = LO;
                     }
                     Serial.print("\n");
                  }
               }
               
               for (int i=5; i<9; i++) 
               {
                  if (taskarray[0][i] == 0xFEFE)
                  {
                     Serial.print("1");
                  }
                  else 
                  {
                     Serial.print("0");
                  }
                  //Serial.print(taskarray[0][i]);
                  
               }
               Serial.print("\n");
               
               // rep speed
               taskarray[0][17] = taskarray[0][5];
               taskarray[0][18] = taskarray[0][6];
               taskarray[0][19] = taskarray[0][7];
               taskarray[0][20] = taskarray[0][8];
               
               
            }break;
               
            case 0xC0: // Richtung
            {
               //  Serial.print("Richtung b 17: ");
               //  Serial.println(buffer[17]);
               
               // Adresse mitgeben
               taskarray[0][0] = tritarray[buffer[8]];
               taskarray[0][1] = tritarray[buffer[9]];
               taskarray[0][2] = tritarray[buffer[10]];
               taskarray[0][3] = tritarray[buffer[11]];
               
               eepromadressearray[0][0] = tritarray[buffer[8]];
               eepromadressearray[0][1] = tritarray[buffer[9]];
               eepromadressearray[0][2] = tritarray[buffer[10]];
               eepromadressearray[0][3] = tritarray[buffer[11]];
               
               
               // speed auf 0 setzen
               for (uint8_t i=1;i<4;i++) 
               {
                  // Serial.println("speed_raw 0: HALT");
                  
                  taskarray[0][5+i] = LO;
               }
               
               
               if (buffer[17] == 1)// Richtung Toggeln
               {
                  taskarray[0][5] = HI; 
                  //  lcd.setCursor(15,0);
                  //  lcd.print("T");
                  
                  /*
                   for (uint8_t i=1;i<4;i++)
                   {
                   Serial.println("speed_raw 0: HALT");
                   
                   taskarray[0][5+i] = LO;
                   }
                   */
               }
               
               else if (buffer[17] == 0)
               {
                  taskarray[0][5] = LO; 
                  //  lcd.setCursor(15,0);
                  //  lcd.print(" ");
                  
               }
               
               // repetition speed 0
               taskarray[0][17] = taskarray[0][5]; // auch richtung
               taskarray[0][18] = taskarray[0][6];
               taskarray[0][19] = taskarray[0][7];
               taskarray[0][20] = taskarray[0][8];
               
               // repetition address
               taskarray[0][12] = taskarray[0][0] ;
               taskarray[0][13] = taskarray[0][1] ;
               taskarray[0][14] = taskarray[0][2] ;
               taskarray[0][15] = taskarray[0][3] ;
               
               
            }break;
               
            case 0xD0: // Funktion
            {
               Serial.print("D0 Funktion b16: ");
               Serial.println(buffer[16]);
               
               // Adresse mitgeben
               taskarray[0][0] = tritarray[buffer[8]];
               taskarray[0][1] = tritarray[buffer[9]];
               taskarray[0][2] = tritarray[buffer[10]];
               taskarray[0][3] = tritarray[buffer[11]];
               
               // repetition address
               taskarray[0][12] = taskarray[0][0] ;
               taskarray[0][13] = taskarray[0][1] ;
               taskarray[0][14] = taskarray[0][2] ;
               taskarray[0][15] = taskarray[0][3] ;
               
               eepromadressearray[0][0] = tritarray[buffer[8]];
               eepromadressearray[0][1] = tritarray[buffer[9]];
               eepromadressearray[0][2] = tritarray[buffer[10]];
               eepromadressearray[0][3] = tritarray[buffer[11]];
               
               
               if (buffer[16] & 0x01)
               {
                  Serial.println("D0 Funktion HI");
                  taskarray[0][4] = HI;
                  taskarray[0][16] = HI;
                  //  lcd.setCursor(12,1);
                  //  lcd.print("ON ");
                  
                  
               }
               else
               {
                  Serial.println("D0 Funktion LO");
                  taskarray[0][4] = LO;
                  taskarray[0][16] = LO;
                  //  lcd.setCursor(12,1);
                  //  lcd.print("OFF");
                  
               }
               
               
            }break;
               
            case 0xE0: // Pause
            {
               Serial.print("E0 Pause b19: ");
               Serial.println(buffer[19]);
               pause = buffer[19];
               
               
            }break;
/*
            case 0xE1: // Timerintervall
            {
               Serial.print("E1 timerintervall b19: ");
               Serial.println(buffer[18]);
               timerintervall = buffer[18];
               
               
            }break;
 */              
               
               
               
            case 0xA1: // Lok 1
            {
               taskarray[1][0] = tritarray[buffer[8]];
               taskarray[1][1] = tritarray[buffer[9]];
               taskarray[1][2] = tritarray[buffer[10]];
               taskarray[1][3] = tritarray[buffer[11]];
               
               eepromadressearray[1][0] = tritarray[buffer[8]];
               eepromadressearray[1][1] = tritarray[buffer[9]];
               eepromadressearray[1][2] = tritarray[buffer[10]];
               eepromadressearray[1][3] = tritarray[buffer[11]];
               
               Serial.println("\nLok 1: ");
               
               Serial.println(buffer[8]);
               Serial.println(buffer[9]);
               Serial.println(buffer[10]);
               Serial.println(buffer[11]);
               
               uint8_t eeprompos = 0x08;
               EEPROM.update(eeprompos++,buffer[8]);
               delay(10);
               EEPROM.update(eeprompos++,buffer[9]);
               delay(10);
               EEPROM.update(eeprompos++,buffer[10]);
               delay(10);
               EEPROM.update(eeprompos++,buffer[11]);
               delay(10);
               
               
               // repetition adresse
               taskarray[1][12] = taskarray[1][0];
               taskarray[1][13] = taskarray[1][1];
               taskarray[1][14] = taskarray[1][2];
               taskarray[1][15] = taskarray[1][3];
               
               
               Serial.print("Lok 1: ");
               Serial.println(buffer[16]);
               
               //   EEPROM.update(0,eepromadressearray);  
               
            }break;
               
            case 0xB1: // speed 1
            {
               taskarray[1][0] = tritarray[buffer[8]];
               taskarray[1][1] = tritarray[buffer[9]];
               taskarray[1][2] = tritarray[buffer[10]];
               taskarray[1][3] = tritarray[buffer[11]];
               
               eepromadressearray[1][0] = tritarray[buffer[8]];
               eepromadressearray[1][1] = tritarray[buffer[9]];
               eepromadressearray[1][2] = tritarray[buffer[10]];
               eepromadressearray[1][3] = tritarray[buffer[11]];
               
               
               // repetition adresse
               taskarray[1][12] = taskarray[1][0] ;
               taskarray[1][13] = taskarray[1][1] ;
               taskarray[1][14] = taskarray[1][2] ;
               taskarray[1][15] = taskarray[1][3] ;
               //Serial.print("usbtaskask 0xB0");
               //     taskarray[0][4] = tritarray[(buffer[16] & 0x01)]; // Licht, bit 0
               
               uint8_t speed_raw = buffer[17]; // 0: halt 1: richtung 2-5: speed
               uint8_t speed_red = 0;
               Serial.print("speed_raw 0: ");
               Serial.println(speed_raw);
               //  lcd.setCursor(0,0);
               ////  lcd.print("Lok0");
               if (speed_raw < 10)
               {
                  //  lcd.print(" ");
               }
               else
               {
                  // //  lcd.print("speed ");
               }
               //  lcd.print(speed_raw);
               
               
               if (speed_raw < 2) // stillstand oder Richtungswachsel
               {
                  
                  for (uint8_t i=0;i<4;i++)
                  {
                     Serial.println("speed_raw 1: HALT");
                     //Serial.println(speed_raw);
                     
                     taskarray[1][5+i] = LO;
                  }
                  if (speed_raw == 1)
                  {
                     Serial.println("speed_raw 0: WENDEN");
                     taskarray[1][5] = HI; // richtungswechsel fuer speed = 1
                     
                     
                  }
               }
               else 
               {
                  speed = speed_raw;
                  
                  //   Serial.print("speed 0: ");
                  //   Serial.println(speed);
                  for (uint8_t i=0;i<4;i++)
                  {
                     Serial.print(" i: "); Serial.print(i);
                     Serial.print(" data: ");Serial.print(speed & (1<<i));
                     Serial.print("\n");
                     if (speed & (1<<i))
                     {
                        Serial.print("HI");
                        speedarray[i] = HI; 
                        //taskarray[0][8-i] = HI;
                        taskarray[1][5+i] = HI;
                     }
                     else
                     {
                        Serial.print("LO");
                        speedarray[i] = LO; 
                        taskarray[1][5+i] = LO;
                     }
                     Serial.print("\n");
                  }
               }
               
               // 
               // Ausgabe bin
               /*
                for (int i=5; i<9; i++) 
                {
                if (taskarray[1][i] == 0xFEFE)
                {
                Serial.print("1");
                }
                else 
                {
                Serial.print("0");
                }
                //Serial.print(taskarray[1][i]);
                
                }
                Serial.print("\n");
                */
               // rep speed
               taskarray[1][17] = taskarray[1][5];
               taskarray[1][18] = taskarray[1][6];
               taskarray[1][19] = taskarray[1][7];
               taskarray[1][20] = taskarray[1][8];
               
               
               
            }break;
               
               
            case 0xC1: // richtung 1
            {
               //  Serial.print("Richtung b 17: ");
               //  Serial.println(buffer[17]);
               
               // Adresse mitgeben
               taskarray[1][0] = tritarray[buffer[8]];
               taskarray[1][1] = tritarray[buffer[9]];
               taskarray[1][2] = tritarray[buffer[10]];
               taskarray[1][3] = tritarray[buffer[11]];
               
               eepromadressearray[1][0] = tritarray[buffer[8]];
               eepromadressearray[1][1] = tritarray[buffer[9]];
               eepromadressearray[1][2] = tritarray[buffer[10]];
               eepromadressearray[1][3] = tritarray[buffer[11]];
               
               // repetition adresse
               taskarray[1][12] = taskarray[1][0] ;
               taskarray[1][13] = taskarray[1][1] ;
               taskarray[1][14] = taskarray[1][2] ;
               taskarray[1][15] = taskarray[1][3] ;
               
               
               // speed auf 0 setzen
               for (uint8_t i=1;i<4;i++) 
               {
                  // Serial.println("speed_raw 0: HALT");
                  
                  taskarray[1][5+i] = LO;
               }
               
               if (buffer[17] == 1)// Richtung Toggeln
               {
                  taskarray[1][5] = HI; 
                  //  lcd.setCursor(15,0);
                  //  lcd.print("T");
                  
                  /*
                   for (uint8_t i=1;i<4;i++)
                   {
                   Serial.println("speed_raw 0: HALT");
                   
                   taskarray[0][5+i] = LO;
                   }
                   */
               }
               
               else if (buffer[17] == 0)
               {
                  taskarray[1][5] = LO; 
                  //  lcd.setCursor(15,0);
                  //  lcd.print(" ");
                  
               }
               
               // repetition speed 0
               taskarray[1][17] = taskarray[1][5];
               taskarray[1][18] = taskarray[1][6];
               taskarray[1][19] = taskarray[1][7];
               taskarray[1][20] = taskarray[1][8];
               
            }break;
               
            case 0xD1:
            {
               taskarray[1][0] = tritarray[buffer[8]];
               taskarray[1][1] = tritarray[buffer[9]];
               taskarray[1][2] = tritarray[buffer[10]];
               taskarray[1][3] = tritarray[buffer[11]];
               
               eepromadressearray[1][0] = tritarray[buffer[8]];
               eepromadressearray[1][1] = tritarray[buffer[9]];
               eepromadressearray[1][2] = tritarray[buffer[10]];
               eepromadressearray[1][3] = tritarray[buffer[11]];
               
               
               
               // repetition adresse
               taskarray[1][12] = taskarray[1][0] ;
               taskarray[1][13] = taskarray[1][1] ;
               taskarray[1][14] = taskarray[1][2] ;
               taskarray[1][15] = taskarray[1][3] ;
               
               
               Serial.print("D1 Funktion b16: ");
               Serial.println(buffer[16]);
               
               if (buffer[16] & 0x01)
               {
                  Serial.println("D0 Funktion HI");
                  taskarray[1][4] = HI;
                  taskarray[1][16] = HI;
               }
               else
               {
                  Serial.println("D0 Funktion LO");
                  taskarray[1][4] = LO;
                  taskarray[1][16] = LO;
               }
            }break;
               
               // Lok 2
            case 0xA2: // address
            {
               
               
               taskarray[2][0] = tritarray[buffer[8]];
               taskarray[2][1] = tritarray[buffer[9]];
               taskarray[2][2] = tritarray[buffer[10]];
               taskarray[2][3] = tritarray[buffer[11]];
               
               
               eepromadressearray[2][0] = tritarray[buffer[8]];
               eepromadressearray[2][1] = tritarray[buffer[9]];
               eepromadressearray[2][2] = tritarray[buffer[10]];
               eepromadressearray[2][3] = tritarray[buffer[11]];
               
               Serial.println("\nLok 2: ");
               
               Serial.println(buffer[8]);
               Serial.println(buffer[9]);
               Serial.println(buffer[10]);
               Serial.println(buffer[11]);
               
               uint8_t eeprompos = 0x10;
               EEPROM.update(eeprompos++,buffer[8]);
               delay(10);
               EEPROM.update(eeprompos++,buffer[9]);
               delay(10);
               EEPROM.update(eeprompos++,buffer[10]);
               delay(10);
               EEPROM.update(eeprompos++,buffer[11]);
               delay(10);
               
               
               // repetition
               taskarray[2][12] = taskarray[2][0] ;
               taskarray[2][13] = taskarray[2][1] ;
               taskarray[2][14] = taskarray[2][2] ;
               taskarray[2][15] = taskarray[2][3] ;
               
               //          EEPROM.update(0,eepromadressearray);  
            }break;
               
            case 0xB2: // speed
            {
               
               // Adresse mitgeben
               taskarray[2][0] = tritarray[buffer[8]];
               taskarray[2][1] = tritarray[buffer[9]];
               taskarray[2][2] = tritarray[buffer[10]];
               taskarray[2][3] = tritarray[buffer[11]];
               
               eepromadressearray[2][0] = tritarray[buffer[8]];
               eepromadressearray[2][1] = tritarray[buffer[9]];
               eepromadressearray[2][2] = tritarray[buffer[10]];
               eepromadressearray[2][3] = tritarray[buffer[11]];
               
               // repetition address
               taskarray[2][12] = taskarray[2][0] ;
               taskarray[2][13] = taskarray[2][1] ;
               taskarray[2][14] = taskarray[2][2] ;
               taskarray[2][15] = taskarray[2][3] ;
               
               //Serial.print("usbtaskask 0xB0");
               //     taskarray[2][4] = tritarray[(buffer[16] & 0x01)]; // Licht, bit 0
               
               uint8_t speed_raw = buffer[17]; // 0: halt 1: richtung 2-5: speed
               uint8_t speed_red = 0;
               Serial.print("speed_raw 0: ");
               Serial.println(speed_raw);
               
               
               if (speed_raw < 2) // stillstand oder Richtungswachsel
               {
                  
                  for (uint8_t i=0;i<4;i++)
                  {
                     //Serial.println("speed_raw 0: HALT");
                     //Serial.println(speed_raw);
                     
                     taskarray[2][5+i] = LO;
                  }
                  if (speed_raw == 1)
                  {
                     Serial.println("speed_raw 0: WENDEN");
                     taskarray[2][5] = HI; // richtungswechsel fuer speed = 1
                     
                     
                  }
               }
               else 
               {
                  speed = speed_raw;
                  
                  //                Serial.print("speed 0: ");
                  //                Serial.println(speed);
                  for (uint8_t i=0;i<4;i++)
                  {
                     //                   Serial.print(" i: "); Serial.print(i);
                     //                   Serial.print(" data: ");Serial.print(speed & (1<<i));
                     //                   Serial.print("\n");
                     if (speed & (1<<i))
                     {
                        //                     Serial.print("HI");
                        speedarray[i] = HI; 
                        taskarray[2][5+i] = HI;
                     }
                     else
                     {
                        //                   Serial.print("LO");
                        speedarray[i] = LO; 
                        taskarray[2][5+i] = LO;
                     }
                     //                Serial.print("\n");
                  }
               }
               /*
                for (int i=5; i<9; i++) 
                {
                if (taskarray[2][i] == 0xFEFE)
                {
                Serial.print("1");
                }
                else 
                {
                Serial.print("0");
                }
                //Serial.print(taskarray[2][i]);
                
                }
                Serial.print("\n");
                */
               // rep speed
               taskarray[2][17] = taskarray[2][5];
               taskarray[2][18] = taskarray[2][6];
               taskarray[2][19] = taskarray[2][7];
               taskarray[2][20] = taskarray[2][8];
               
               
            }break;
               
            case 0xC2:
            {
               //  Serial.print("Richtung b 17: ");
               //  Serial.println(buffer[17]);
               
               // Adresse mitgeben
               taskarray[2][0] = tritarray[buffer[8]];
               taskarray[2][1] = tritarray[buffer[9]];
               taskarray[2][2] = tritarray[buffer[10]];
               taskarray[2][3] = tritarray[buffer[11]];
               
               eepromadressearray[2][0] = tritarray[buffer[8]];
               eepromadressearray[2][1] = tritarray[buffer[9]];
               eepromadressearray[2][2] = tritarray[buffer[10]];
               eepromadressearray[2][3] = tritarray[buffer[11]];
               
               
               // speed auf 0 setzen
               for (uint8_t i=1;i<4;i++) 
               {
                  // Serial.println("speed_raw 0: HALT");
                  
                  taskarray[2][5+i] = LO;
               }
               
               if (buffer[17] == 1)// Richtung Toggeln
               {
                  taskarray[2][5] = HI; 
                  //  lcd.setCursor(15,0);
                  //  lcd.print("T");
                  
                  /*
                   for (uint8_t i=1;i<4;i++)
                   {
                   Serial.println("speed_raw 0: HALT");
                   
                   taskarray[2][5+i] = LO;
                   }
                   */
               }
               
               else if (buffer[17] == 0)
               {
                  taskarray[2][5] = LO; 
                  //  lcd.setCursor(15,0);
                  //  lcd.print(" ");
                  
               }
               
               // repetition speed 0
               taskarray[2][17] = taskarray[2][5];
               taskarray[2][18] = taskarray[2][6];
               taskarray[2][19] = taskarray[2][7];
               taskarray[2][20] = taskarray[2][8];
               
               // repetition address
               taskarray[2][12] = taskarray[2][0] ;
               taskarray[2][13] = taskarray[2][1] ;
               taskarray[2][14] = taskarray[2][2] ;
               taskarray[2][15] = taskarray[2][3] ;
               
               
            }break;
               
            case 0xD2:
            {
               Serial.print("D0 Funktion b16: ");
               Serial.println(buffer[16]);
               
               // Adresse mitgeben
               taskarray[2][0] = tritarray[buffer[8]];
               taskarray[2][1] = tritarray[buffer[9]];
               taskarray[2][2] = tritarray[buffer[10]];
               taskarray[2][3] = tritarray[buffer[11]];
               
               // repetition address
               taskarray[2][12] = taskarray[2][0] ;
               taskarray[2][13] = taskarray[2][1] ;
               taskarray[2][14] = taskarray[2][2] ;
               taskarray[2][15] = taskarray[2][3] ;
               
               eepromadressearray[2][0] = tritarray[buffer[8]];
               eepromadressearray[2][1] = tritarray[buffer[9]];
               eepromadressearray[2][2] = tritarray[buffer[10]];
               eepromadressearray[2][3] = tritarray[buffer[11]];
               
               
               if (buffer[16] & 0x01)
               {
                  Serial.println("D0 Funktion HI");
                  taskarray[2][4] = HI;
                  taskarray[2][16] = HI;
                  //  lcd.setCursor(12,1);
                  //  lcd.print("ON ");
                  
                  
               }
               else
               {
                  Serial.println("D0 Funktion LO");
                  taskarray[2][4] = LO;
                  taskarray[2][16] = LO;
                  //  lcd.setCursor(12,1);
                  //  lcd.print("OFF");
                  
               }
               
               
            }break;
               
            case 0xE2: // Pause
            {
               Serial.print("E0 Pause b19: ");
               Serial.println(buffer[19]);
               pause = buffer[19];
               
               
            }break;
               
               
         }// switch
         
      } // if localstatus & 0x02
      Serial.println("USB END");
   } // n>0
   
#pragma mark local
   else if (sourcestatus & 0x01)
   {
     // if (digitalReadFast(SOURCECONTROL) == 1)
      for (uint8_t localnum = 0;localnum < ANZLOKALLOKS;localnum++)
      {
         
         loknummer =localnum;
         //loknummer = !loknummer;
          for (uint8_t i=0;i<4;i++)
          {
            // if (tastenadresseA & (1<<i))
            if (lokaladressearray[localnum] & (1<<i))
             {
                taskarray[localnum][i] = tritarray[2];
             }
             else
             {
                taskarray[localnum][i] = tritarray[0];
             }
             
             
          }
         
          
         // repetition address
          taskarray[localnum][12] = taskarray[localnum][0] ;
          taskarray[localnum][13] = taskarray[localnum][1] ;
          taskarray[localnum][14] = taskarray[localnum][2] ;
          taskarray[localnum][15] = taskarray[localnum][3] ;

         // speed
         
         uint8_t speed_raw = localpotarray[localnum] >> 4; // 0: halt 1: richtung 2-5: speed
         if (speed_raw > 0)
         {
            speed_raw += 1; // speed 1 ist Richtungsumschaltung
         }
         if (speed_raw > 15)
         {
            speed_raw = 15;
         }
         
         /*
          if (richtungstatus & (1<<RICHTUNGSTART)) // Richtungswechsel im Gang
          {
            richtungcounter++;
          if (richtungcounter > 4)
          {
            richtungstatus &= ~(1<<RICHTUNGSTART); // Richtungswechsel beenden
          taskarray[localnum][5] = HI; // Richtungbit reset
          richtungcounter = 0;
          }
          }
          */
         
         uint8_t speed_red = 0;
  //       Serial.print("local speed_raw 0: ");
  //       Serial.println(speed_raw);
  //       //  lcd.setCursor(0,0);
  //       //  lcd.print("Lok0");
 /*        
         lcd_gotoxy(0, 0);
         lcd_puts("Lok");
         lcd_putint1(localnum);
         lcd_putc(' ');
         lcd_puthex(speed_raw);
         lcd_putc(' ');
         if (speed_raw < 10)
         {
            lcd_puts("min");
  //          //  lcd.print(" ");
         }
         else
         {
            lcd_putint(speed_raw);
            // //  lcd.print("speed ");
         }
//         //  lcd.print(speed_raw);
         
 */        
         
#pragma mark local speed
         if (speed_raw < 2) // stillstand oder Richtungswachsel
         {
            
            // speed auf 0 setzen
            for (uint8_t i=0;i<4;i++)
            {
               //Serial.println("speed_raw 0: HALT");
               //Serial.println(speed_raw);
               
               taskarray[localnum][5+i] = LO;
               
            }
            
            // tritt nicht auf, 1 wird uebersprungen
            if (speed_raw == 1)
            {
               Serial.println("speed_raw 0: WENDEN");
               taskarray[localnum][5] = HI; // richtungswechsel fuer speed = 1
               
            }
         } // if speed_raw < 2
         else 
         {
            uint8_t speed_full = localpotarray[localnum] ; //8-bit Wert, 
            speed = speed_raw;
            
            // speed setzen
            
//            Serial.print("speed 0: ");
 //           Serial.println(speed);
            for (uint8_t i=0;i<4;i++)
            {
 //              Serial.print(" i: "); Serial.print(i);
 //              Serial.print(" data: ");Serial.print(speed & (1<<i));
 //              Serial.print("\n");
               if (speed & (1<<i))
               {
//                 Serial.print("1");
                  speedarray[i] = HI; 
                  taskarray[localnum][5+i] = HI;
               }
               else
               {
    //              Serial.print("0");
                  speedarray[i] = LO; 
                  taskarray[localnum][5+i] = LO;
               }
    //           Serial.print("\n");
            }
         } // speed_raw >= 2
         
         
      /*   
         for (int i=5; i<9; i++) 
         {
            if (taskarray[localnum][i] == 0xFEFE)
            {
               Serial.print("1");
            }
            else 
            {
               Serial.print("0");
            }
            //Serial.print(taskarray[localnum][i]);
            
         }
         Serial.print("\n");
         */
         // rep speed
         /*
         taskarray[localnum][17] = taskarray[localnum][5];
         taskarray[localnum][18] = taskarray[localnum][6];
         taskarray[localnum][19] = taskarray[localnum][7];
         taskarray[localnum][20] = taskarray[localnum][8];
         */
   
         // Richtung
          
         
           if (lokalcodearray[localnum] & 0x02) // von debounce,  Richtungsimpuls, bit 1
           {
              
              if(!(richtungstatus & (1<<RICHTUNGSTART))) // start richtungtask
              {
                 Serial.println("RICHTUNGSTART gesetzt");
                 richtungstatus |= (1<<RICHTUNGSTART); // Beginn Richtungswechsel
                 richtungcounter = 0;
                  
                    
              }
              else
              {
                 //richtungcounter++;
              }
              
              
              // speed auf 0 setzen
              for (uint8_t i=1;i<4;i++) 
              {
                  //Serial.println("speed_raw 0: HALT");
                 
                 taskarray[localnum][5+i] = LO;
              }
          
              taskarray[localnum][5] = HI; // Richtungbit set
              
              lokalstatus |= (1<<LOKALRICHTUNGBIT0);
         
               
              sincelocalrichtung = 0;
              
           }
           else //if ((lokalstatus & (1<<LOKALRICHTUNGBIT0)) && (sincelocalrichtung > 1000)) // nach 0.5s zuruecksetzen
           
           {
              if ((lokalstatus & (1<<LOKALRICHTUNGBIT0)) && (sincelocalrichtung > 4000)) // nach 0.5s zuruecksetzen
              {
                 lokalstatus &= ~(1<<LOKALRICHTUNGBIT0);
                 sincelocalrichtung = 0;
                 taskarray[localnum][5] = LO;
                 //           //  lcd.setCursor(6,0);
                 //           //  lcd.print("W1");
                 
                 
              }
           }
         
         
         // repetition speed 
          taskarray[localnum][17] = taskarray[localnum][5]; // auch richtung
          taskarray[localnum][18] = taskarray[localnum][6];
          taskarray[localnum][19] = taskarray[localnum][7];
          taskarray[localnum][20] = taskarray[localnum][8];

         // Funktion, bit 0
         if (lokalcodearray[localnum] & 0x01)
         {
            taskarray[localnum][4] = HI; 
            taskarray[localnum][16] = HI; // rep
//            //  lcd.setCursor(12,1);
//            //  lcd.print("ON ");
            
         }
         else
         {
            taskarray[localnum][4] = LO;
            taskarray[localnum][16] = LO; // rep
 //           //  lcd.setCursor(12,1);
 //           //  lcd.print("OFF");
            
         }
      }
   } // local
   

#pragma mark sincewegbuffer 
   
   if ((sincewegbuffer > 1000))// && (usbtask == SET_WEG)) // naechster Schritt
   {
      sendbuffer[10] = 0xAB;
      sendbuffer[12] = emitter;
      
      n = RawHID.send(sendbuffer, 100);
      if (n > 0) 
      {
         //        Serial.print(F("Transmit packet "));
         //        Serial.println(packetCount );
         packetCount = packetCount + 1;
      } else 
      {
         Serial.println(F("Unable to transmit packet"));
      }

      sincewegbuffer = 0;
      
      //     Serial.print(" abschnittindex: ");
      //     Serial.print(abschnittindex);
      
      //     Serial.print(" aktuellepos index: ");
      //    Serial.println(aktuellepos.index);
      
      //if (schrittecount == 0)
      if (usbtask == 0)
      {
         
         
         // **************
        // if ((schrittecount == 0)  && (!(usbtask == END_WEG)))
         
          
      }  
         
         // **************
         
 
         
         //if ((schrittecount < anzschritte ) && (wegstatus & (1<<WEG_OK)))//&& ((abschnittindex+1) == aktuellepos.index))
       
   }   

#pragma mark sinceringbuffer  
   
   if ((sinceringbuffer > 32))// && (usbtask == SET_RING)) // naechster Schritt
   {
      sinceringbuffer = 0;
      //     Serial.print(" abschnittindex: ");
      //     Serial.print(abschnittindex);
      
      //     Serial.print(" aktuellepos index: ");
      //    Serial.println(aktuellepos.index);
      
      //if (schrittecount == 0)
      
      
   }
   
   
   
   // every 4 seconds, send a packet to the computer
   if (msUntilNextSend > 4000) 
   {
      msUntilNextSend = msUntilNextSend - 2000;
      
      
      /*
      // first 2 bytes are a signature
      buffer[10] = 0xAB;
      buffer[11] = 0xCD;
      // next 24 bytes are analog measurements
      for (int i=5; i<12; i++) 
      {
         int val = analogRead(i);
         buffer[i * 2 + 2] = highByte(val);
         buffer[i * 2 + 3] = lowByte(val);
      }
      // fill the rest with zeros
      for (int i=26; i<62; i++) 
      {
         buffer[i] = 0;
      }
      // and put a count of packets sent at the end
      buffer[62] = highByte(packetCount);
      buffer[63] = lowByte(packetCount);
      */
      // actually send the packet
      /*
      buffer[10] = 0xAB;
      buffer[12] = emitter;

      n = RawHID.send(buffer, 100);
      if (n > 0) 
      {
 //        Serial.print(F("Transmit packet "));
 //        Serial.println(packetCount );
         packetCount = packetCount + 1;
      } else 
      {
         Serial.println(F("Unable to transmit packet"));
      }
       */
   }
} // loop
