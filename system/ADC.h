// --------------------------------------------------------------------------------
//                                    ADC.h
//                                 ADC routines
//                                 AVR 1 series
// --------------------------------------------------------------------------------
// This file includes functions for use of the ADC module of AVR 1 series
//    Attiny Microcontrollers.  The module is designed for polled operation.
//  The public functions are:
//		ADC_init - initialize the ADC module - modify settings here
//		ADC_start_sample - start a conversion on the given channel
//		ADC_sample_done - check to see if conversion done/ready to read
//		ADC_get_sample_result - read data from the sampling run
//		ADC_take_sample - start a conversion, wait for it to complete, and 
//							return the result
//  The following local functions are included:
//		ADC_pins_init - enable ADC mode of input pins - modify settings here
//  Special Notes:  only supports 8 bit conversions
//  Known Bugs:		None
//
//  Revision History:
//     5/29/18  Nathan Barton      initial revision

#include <avr/io.h>

#ifndef ADC_H_
#define ADC_H_

void ADC_init(void);

void ADC_start_sample(uint8_t channel);
uint8_t ADC_sample_done(void);
uint8_t ADC_get_sample_result(void);
uint8_t ADC_take_sample(uint8_t channel);





#endif /* ADC_H_ */