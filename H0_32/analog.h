/* vim: set sw=8 ts=8 si : */
/*************************************************************************
 Title	:   C include file for analog conversion
 Target:    any AVR device
 Copyright: GPL
***************************************************************************/
#include "ADC.h"


#ifndef ANALOG_H
#define ANALOG_H

extern ADC *adc;

#define ADC_U  0
#define ADC_I  2
#define ADC_POT   4

#define OSZIA 9


extern void init_analog(void);
/* get the result of an analog conversion */
extern int16_t get_analogresult(uint8_t channel);
extern void set_target_adc_val(uint8_t item,int16_t val);
extern uint8_t is_current_limit(void); 
extern int16_t get_dacval(void);
int16_t get_targetvalue(uint8_t channel);
uint8_t get_currentcontrol(void);
extern uint16_t readPot(uint8_t pin);
//extern static void control_loop(void);
extern void set_target_U(uint16_t U_target);
extern void dec_targetvalue(uint8_t channel, uint16_t dec); // targetvalue decrementieren um dec
extern void inc_targetvalue(uint8_t channel, uint16_t dec); // targetvalue incrementieren um dec


extern ADC::Sync_result result;

extern int16_t get_analogresult(uint8_t channel);
extern int16_t adc_u_to_disp(int16_t adcunits);
extern void int_to_dispstr(uint16_t inum,char *outbuf,int8_t decimalpoint_pos);
#endif /* ANALOG_H */
