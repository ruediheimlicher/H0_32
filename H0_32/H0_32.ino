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


// Set parameters

#define TEST 1
// Include application, user and local libraries
// !!! Help http://bit.ly/2CL22Qp


// Define structures and classes


// Define variables and constants
//#define STARTWERT  2840 // Mitte
#define STARTWERT  2080 // Nullpunkt
#define MAXWERT  4096 // Nullpunkt

#define TAKT_PIN 0
#define OUT_PIN 1

#define OSZI_PULS_A        8
#define OSZI_PULS_B        9


byte buffer[64];
elapsedMillis msUntilNextSend;
unsigned int packetCount = 0;

volatile uint8_t usbtask = 0;

volatile uint8_t teensytask = 0;

volatile uint16_t aktualcommand = 0;

volatile uint16_t commandarray0[26] = {0};
volatile uint16_t commandarray1[26] = {0};
volatile uint16_t commandarray2[26] = {0};

volatile uint16_t taskarray[8][26] = {0};



volatile uint16_t adressearray[4];
volatile uint16_t speedarray[5];

volatile uint8_t speed = 0;


//let GET_U:UInt8 = 0xA2
//let GET_I:UInt8 = 0xB2

// sinus
elapsedMillis sinms;
elapsedMillis sinceblink;


float sinpos = 0;
#define pi 3.14
#define SIN_START   0xE0
#define SIN_STOP   0xE1


#define LOOPLED 13

char* buffercode[4] = {"BUFFER_FAIL","BUFFER_SUCCESS", "BUFFER_FULL", "BUFFER_EMPTY"};

// Prototypes
// !!! Help: http://bit.ly/2l0ZhTa

#define HI     0xFEFE  // 111111101111111 // 1111111011111110
#define LO     0x0202  // 100000001000000 // 0000001000000010
#define OPEN   0x02FE  // 111111101000000 // 0000001011111110

#define TIMERINTERVALL  13
// Utilities
elapsedMillis sinceringbuffer;


elapsedMillis sincewegbuffer;

uint16_t abschnittindex = 0; // aktuelles Element in positionsarray

// Create an IntervalTimer object 
IntervalTimer              paketTimer;
volatile uint16_t          timerintervall = TIMERINTERVALL;

volatile uint8_t     paketcounter = 0;

volatile uint8_t  commandpos = 0; // pos im command
volatile uint8_t bytepos = 0; // pos im Ablauf

uint16_t tritarray[] = {LO,OPEN,HI};

int achse0_startwert=0;

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


void pakettimerfunction() 
{ 
   
   digitalWriteFast(TAKT_PIN, !digitalReadFast(TAKT_PIN)); // LO, OFF 
   aktualcommand = commandarray0[bytepos];
   if ((bytepos) == 0)
   {
      OSZI_A_LO();
   }
   if (aktualcommand & (1<<commandpos))
   {
      digitalWriteFast(OUT_PIN,HIGH);
   }
   else
   {
      digitalWriteFast(OUT_PIN,LOW);
   }
   if (commandpos < 15)
   {
      commandpos++;
   }
   else
   {
      commandpos = 0;
      
      digitalWriteFast(OUT_PIN,LOW);
      bytepos++;
      if (bytepos >= 26)
      {
         bytepos = 0;
         OSZI_A_HI();
      }
   
   }
   
   paketcounter++;
   
}


// Add setup code
void setup()
{
   Serial.begin(9600);

   analogWriteResolution(16); // 32767
   
   paketTimer.begin(pakettimerfunction,timerintervall);
   paketTimer.priority(0);
   
   pinMode(LOOPLED, OUTPUT);
   
   // FTM0   Pins: 5, 6, 9, 10, 20, 21, 22, 23
   // FTM1   3, 4   
   // FTM2   25, 32
   analogWriteFrequency(5, 50);
   Serial.println(F("RawHID H0"));
 
   #define  achse0_PIN 5
   #define  achse1_PIN 6
   #define  achse2_PIN 9
   #define  achse3_PIN 10
   
   pinMode(TAKT_PIN, OUTPUT);
   digitalWriteFast(TAKT_PIN, LOW); // LO, OFF 

   pinMode(OUT_PIN, OUTPUT);
   digitalWriteFast(OUT_PIN, LOW); // LO, OFF 

   
   pinMode(OSZI_PULS_A, OUTPUT);
   digitalWriteFast(OSZI_PULS_A, HIGH); 
   pinMode(OSZI_PULS_B, OUTPUT);
   digitalWriteFast(OSZI_PULS_B, HIGH); 
   
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
   
   aktualcommand = OPEN;
   
}

// Add loop code
void loop()
{
   if (sinceblink > 1000)
   {
      sinceblink = 0;
      digitalWriteFast(LOOPLED, !digitalReadFast(LOOPLED));
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
       */
      if (speed > 15)
      {
            speed = 0;
      }
      
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
       speed++;
      
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
   }
   #pragma mark USB
   int n;
   n = RawHID.recv(buffer, 10); // 0 timeout = do not wait
   if (n > 0) 
   {
      // the computer sent a message.  Display the bits
      // of the first byte on pin 0 to 7.  Ignore the
      // other 63 bytes!
      //Serial.print(F("Received packet, erstes byte: "));
      //Serial.println((int)buffer[0]);
      for (int i=0; i<8; i++) 
      {
  //       int b = buffer[0] & (1 << i);
  //       Serial.print((int)buffer[i]);
  //       Serial.print("\t");
         //digitalWrite(i, b);
      }
 //     Serial.println();
  //     Serial.print(hb);
 //     Serial.print("\t");
 //     Serial.print(lb);
 //     Serial.println();
      
      usbtask = buffer[0];
      Serial.println(" ");
      Serial.print("******************  usbtask *** ");
      printHex8(usbtask);
      for (int i=0; i<24; i++) 
      {
         Serial.print(buffer[i]);
         Serial.print(" ");
       }
      Serial.print("\n");
      
      if (timerintervall != buffer[18])
      {
         Serial.print("timerintervall changed\n");
         
         paketTimer.update(  buffer[18]); 
         timerintervall = buffer[18];
         Serial.print(timerintervall);
      }
      #pragma mark TASK 
      //usbtask = SET_0;
      switch (usbtask)
      {
            
         case 0xA0:
         {
            commandarray0[0] = tritarray[buffer[8]];
            commandarray0[1] = tritarray[buffer[9]];
            commandarray0[2] = tritarray[buffer[10]];
            commandarray0[3] = tritarray[buffer[11]];
            
            Serial.print("richtung: ");
            Serial.println(buffer[16]);

            commandarray0[4] = tritarray[buffer[16]];
            
            commandarray0[12] = commandarray0[0];
            commandarray0[13] = commandarray0[1];
            commandarray0[14] = commandarray0[2];
            commandarray0[15] = commandarray0[3];
            commandarray0[16] = commandarray0[4];
            Serial.print("speed 0: ");
            Serial.println(buffer[17]);
            uint8_t speed = buffer[17];
            for (uint8_t i=0;i<4;i++)
            {
               Serial.print(" i: "); Serial.print(i);
               Serial.print(" data: ");Serial.print(speed & (1<<i));
               Serial.print("\n");
               if (speed & (1<<i))
               {
                  Serial.print("HI");
                  speedarray[i] = HI; 
                  commandarray0[5+i] = HI;
                  
               }
               else
               {
                  Serial.print("LO");
                  speedarray[i] = LO; 
                  commandarray0[5+i] = LO;
               }
               Serial.print("\n");
            }
            commandarray0[17] = commandarray0[5];
            commandarray0[18] = commandarray0[6];
            commandarray0[19] = commandarray0[7];
            commandarray0[20] = commandarray0[8];

  
            
         }break;
 
         case 0xA1: // Lok 1
         {
            commandarray1[0] = tritarray[buffer[8]];
            commandarray1[1] = tritarray[buffer[9]];
            commandarray1[2] = tritarray[buffer[10]];
            commandarray1[3] = tritarray[buffer[11]];
            
            Serial.print("richtung: ");
            Serial.println(buffer[16]);
            
            commandarray0[4] = tritarray[buffer[16]];
            
            commandarray1[12] = commandarray0[0];
            commandarray1[13] = commandarray0[1];
            commandarray1[14] = commandarray0[2];
            commandarray1[15] = commandarray0[3];
            commandarray1[16] = commandarray0[4];
            Serial.print("speed 1: ");
            Serial.println(buffer[17]);
            uint8_t speed = buffer[17];
            for (uint8_t i=0;i<4;i++)
            {
               Serial.print(" i: "); Serial.print(i);
               Serial.print(" data: ");Serial.print(speed & (1<<i));
               Serial.print("\n");
               if (speed & (1<<i))
               {
                  Serial.print("HI");
                  speedarray[i] = HI; 
                  commandarray0[5+i] = HI;
                  
               }
               else
               {
                  Serial.print("LO");
                  speedarray[i] = LO; 
                  commandarray0[5+i] = LO;
               }
               Serial.print("\n");
            }
            commandarray1[17] = commandarray0[5];
            commandarray1[18] = commandarray0[6];
            commandarray1[19] = commandarray0[7];
            commandarray1[20] = commandarray0[8];
            
            
            
         }break;
  

 
           
       
      }// switch
   } // n>0
   
   
   if (teensytask == SIN_START)
   {
      if (sinms > 5)
      {
         sinms = 0;
         //float tempsin = 0x800 + achse0_start + 1000 * sin(sinpos/180*pi);
        float tempsin0 = 0xCCC   + 0x400 * sin(0.5*sinpos/180*pi);
         Serial.print("Sin Pot0: ");
         //Serial.print((int)sinpos);
         Serial.print("\t");
         Serial.print((int)tempsin0);
         
         
         analogWrite(6, (int)tempsin0);
         
         float tempsin1 = 0xCCC  + 0x400 * sin((0.2*sinpos)/180*pi);
         analogWrite(achse3_PIN, (int)tempsin1);
         Serial.print("\t");
         Serial.print((int)tempsin1);
      
         Serial.println();
         sinpos += 1;
         
      }
   }

#pragma mark sincewegbuffer 
   
   if ((sincewegbuffer > 128))// && (usbtask == SET_WEG)) // naechster Schritt
   {
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
   }
} // loop
