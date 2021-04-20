/* vim: set sw=8 ts=8 si : */
/*********************************************
* Author: Guido Socher, Copyright: GPL 
*
* Digital analog conversion of channel ADC0 and ADC1 in
* free running mode. 
**********************************************/
#include "Arduino.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <inttypes.h>
#include <stdlib.h>
//#include "dac.h"
//#include "uart.h"
//#include "hardware_settings.h"

#include "analog.h"


#define TONE A7

extern int val;
extern volatile uint8_t inbuffer[64];
extern volatile uint8_t outbuffer[64];
//extern int ADC_U, ADC_I;
extern  int readPin;
extern int U_Pot;
extern volatile int16_t analog_result[8]; 

ADC::Sync_result result;

//debug LED:
// set output to VCC, red LED off
//#define LEDOFF PORTD|=(1<<PORTD0)
// set output to GND, red LED on
//#define LEDON PORTD&=~(1<<PORTD0)
// to test the state of the LED
//#define LEDISOFF PORTD&(1<<PORTD0)
static volatile uint8_t currentcontrol=1; // 0=voltage control, otherwise current control
// adc measurement results (11bit ADC):
extern float sinval;
volatile int16_t raw_analog_u_result[8]; 

volatile int16_t analog_result[8]; 
// target_val is the value that is requested (control loop calibrates to this).
// We use the same units a the ADC produces.
static volatile int16_t target_val[2];  // datatype int is 16 bit

static volatile int16_t dac_val=800; // the current dac setting

static void control_loop(void);

// https://forum.pjrc.com/threads/25I_RESISTOR
//532 -ADC-library-update-now-with-support-for-Teensy-3-1

void init_analog(void) 
{
 	analog_result[0]=50; // I
	analog_result[1]=2000;  // U
	target_val[0]=100; // initialize to zero, I, kein Strom
	target_val[1]=2000; // initialize to 5000, U
   analogWriteFrequency(4, 375000);
   
   adc->adc0->setAveraging(2); // set number of averages 
   adc->adc0->setResolution(12); // set bits of resolution
   adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);
   adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
   adc->adc0->setReference(ADC_REFERENCE::REF_3V3);
   
   
   adc->adc0->enableInterrupts(ADC_0);
    
    
   adc->startSynchronizedContinuous(ADC_U, ADC_I); // 
   delay(100);
   
   
 }

// https://platformio.org/lib/show/355/Teensy_ADC/examples?file=analogContinuousRead.ino

void adc0_isr(void) 
{
   //digitalWriteFast(OSZIA,LOW);
   result = adc->readSynchronizedContinuous();
   
   //analog_result[0] = adc->analogReadContinuous(ADC_0);// I
   analog_result[0] = (uint16_t)result.result_adc0;// I
   
   //analog_result[1] = adc->analogReadContinuous(ADC_1);// U
   analog_result[1] = (uint16_t)result.result_adc1;;// U
   
   //U_Pot = adc->analogRead(A9,ADC_0);
//   val = analog_result[1]; // U
//   outbuffer[0] = 0;
//   outbuffer[1] = (val & 0xFF00) >> 8;
//   outbuffer[2] = val & 0x00FF;
   //digitalWriteFast(OSZIA,LOW);
  /*
   if (analog_result[0] > SH_CIR_PROT)
   {
      dac_val=400;
      //dac(dac_val);
      currentcontrol=20;
      return;
   }
*/
 //  control_loop(); // < 1us
   //digitalWriteFast(OSZIA,HIGH);

}

void adc1_isr(void) 
{
   //analog_result[1] = adc->analogReadContinuous(ADC_1);
   
   //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
   
}


// TUX
int16_t get_dacval(void) 
{
	return(dac_val);
}

uint8_t get_currentcontrol(void)
{
   return currentcontrol;
}

uint8_t is_current_limit(void) 
{
   // return 1 if current control loop active
   if (currentcontrol)
   {
      return(1);
   }
   return(0);
}

/* set the target adc value for the control loop
 * values for item: 1 = u, 0 = i, units must be of the same as the values 
 * from the dac.
 */
void set_target_adc_val(uint8_t item,int16_t val) 
{
   // here we can directly write to target_val 
   target_val[item]=val;
}

int16_t get_analogresult(uint8_t channel) 
{
	return(analog_result[channel]);
}

int16_t get_targetvalue(uint8_t channel) //
{
   return target_val[channel];
}

void set_targetvalue(uint8_t channel, uint16_t val)
{
   target_val[channel] = val;
}

void inc_targetvalue(uint8_t channel, uint16_t inc) // targetvalue incrementieren um inc
{
   if (target_val[channel] < (0xFDD))
   {
      target_val[channel] += inc;
   }
}

void dec_targetvalue(uint8_t channel, uint16_t dec) // targetvalue decrementieren um dec
{
   if (target_val[channel] > (0x0F))
   {
      target_val[channel] -= dec;
   }
}
     
     // the control loop changes the dac:

     
uint16_t readPot(uint8_t pin)
{
   return adc->analogRead(pin);
}



// Convert an integer which is representing a float into a string.
// Our display is always 4 digits long (including one
// decimal point position). decimalpoint_pos defines
// after how many positions from the right we set the decimal point.
// The resulting string is fixed width and padded with leading space.
//
// decimalpoint_pos=2 sets the decimal point after 2 pos from the right:
// e.g 74 becomes "0.74"
// The integer should not be larger than 999.
// The integer must be a positive number.
// decimalpoint_pos can be 0, 1 or 2
void int_to_dispstr(uint16_t inum,char *outbuf,int8_t decimalpoint_pos)
{
   int8_t i,j;
   char chbuf[8];
   itoa(inum,chbuf,10); // convert integer to string
   i=strlen(chbuf);
   if (i>3) i=3; //overflow protection
   strcpy(outbuf,"   0"); //decimalpoint_pos==0
   if (decimalpoint_pos==1) strcpy(outbuf," 0.0");
   if (decimalpoint_pos==2) strcpy(outbuf,"0.00");
   j=4;
   while(i){
      outbuf[j-1]=chbuf[i-1];
      i--;
      j--;
      if (j==4-decimalpoint_pos){
         // jump over the pre-set dot
         j--;
      }
   }
}





/* the following function will be called when analog conversion is done.
 * It will be called every 13th cycle of the converson clock. At 8Mhz
 * and a ADPS factor of 64 this is: ((8000 kHz/ 64) / 13)= 9.6KHz intervalls
 *
 * We do 4 fold oversampling as explained in atmel doc AVR121.
 */

/*
 
ISR(ADC_vect)
{
	uint8_t i=0;
	uint8_t adlow;
	int16_t currentadc;
	static uint8_t channel=0; 
	static uint8_t chpos=0;
	// raw 10bit values:
        static int16_t raw_analog_u_result[8];  
        int16_t new_analog_u_result=0;
	adlow=ADCL; // read low first !! 
	currentadc=(ADCH<<8)|adlow;
	// toggel the channel between 0 an 1. This will however
	// not effect the next conversion as that one is already
	// ongoing.
	channel^=1;
	// 2.56V ref
	ADMUX=(1<<REFS1)|(1<<REFS0)|channel;
	// channel=1 = U, channel=0 = I
	if (channel==1) // Spannung messen
   {
		raw_analog_u_result[chpos]=currentadc;
		//
		// we do 4 bit oversampling to get 11bit ADC resolution
		chpos=(chpos+1)%4; // rotate over 4 
		//analog_result[1]=0;
		while(i<4)
      {
			new_analog_u_result+=raw_analog_u_result[i];
			i++;
		}
		new_analog_u_result=new_analog_u_result>>1; // 11bit
		// mean value:
		analog_result[1]=(new_analog_u_result+analog_result[1])/2; 
	}
   else // channel==0, Strom messen
   {
		analog_result[0]=currentadc; // 10bit
	}
   
	// short circuit protection does not use the over sampling results
	// for speed reasons.
	// short circuit protection, current is 10bit ADC
	if (channel==0 && currentadc > SH_CIR_PROT)
   {
		dac_val=400;
		//dac(dac_val);
		currentcontrol=20;
		return;
	}
        //
	if (channel==1)
   {
		// only after full measurement cycle
		control_loop();
	}
   else
   {
		uart_poll_getchar_isr();
	}
	// end of interrupt handler
}
*/
